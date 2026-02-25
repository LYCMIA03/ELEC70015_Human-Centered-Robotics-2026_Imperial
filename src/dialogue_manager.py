"""Dialogue pipeline with rule-based decision manager."""

import argparse
import glob
import random
import subprocess
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple, Union

from src.utils.nlu_intent import Intent, IntentClassifier
from src.utils.speech_to_text import load_model, recognize_from_mic, recognize_from_wav

ROBOT_PROMPT_INITIAL_WAV = "voice_data/robot_prompt_*.wav"
ROBOT_PROMPT_CLARIFICATION_WAV = "voice_data/robot_prompt_*.wav"
ROBOT_ACTION_PROCEED_WAV = "voice_data/robot_proceed_*.wav"
ROBOT_ACTION_DECLINE_WAV = "voice_data/robot_leave_*.wav"

# No hardcoded local model path.
STT_MODEL_NAME = "vosk-model-small-en-us-0.15"
STT_LANG = "en-us"


class DecisionOutcome(str, Enum):
    PROCEED = "proceed"
    DECLINE = "decline"
    TERMINATE = "terminate"


def _resolve_wav_candidates(wav_spec: Union[str, Path]) -> List[Path]:
    spec = str(wav_spec)
    if "*" not in spec and "?" not in spec and "[" not in spec:
        return [Path(spec)]
    return [Path(p) for p in sorted(glob.glob(spec)) if Path(p).is_file()]


def sample_wav(wav_spec: Union[str, Path]) -> Path:
    candidates = _resolve_wav_candidates(wav_spec)
    if not candidates:
        raise FileNotFoundError(f"No WAV file matched: {wav_spec}")
    selected_wav = random.choice(candidates)
    print(f"[Sample Voice] spec='{wav_spec}' -> using '{selected_wav}'")
    return selected_wav


def play_wav(wav_path: Union[str, Path]) -> None:
    wav_spec = str(wav_path)
    if "*" in wav_spec or "?" in wav_spec or "[" in wav_spec:
        wav = sample_wav(wav_path)
    else:
        wav = Path(wav_path)
    players = (["aplay", str(wav)], ["ffplay", "-autoexit", "-nodisp", str(wav)])
    last_error: Optional[Exception] = None
    for cmd in players:
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
        except FileNotFoundError as e:
            last_error = e
        except subprocess.CalledProcessError as e:
            last_error = e
    raise RuntimeError("No available WAV player found. Install `aplay` or `ffplay`.") from last_error


def speak_robot(wav_spec: Union[str, Path], play_audio: bool) -> None:
    selected_wav = sample_wav(wav_spec)
    if play_audio:
        play_wav(selected_wav)


def transcribe_wav(wav_path: Union[str, Path]) -> str:
    user_wav = Path(wav_path)
    if not user_wav.is_file():
        raise FileNotFoundError(f"User WAV file not found: {wav_path}")
    # If local cache is missing, Vosk will auto-download this model.
    model = load_model(model_path=None, model_name=STT_MODEL_NAME, lang=STT_LANG)
    return recognize_from_wav(str(user_wav), model).strip()


def transcribe_mic(device: Optional[int] = None) -> str:
    model = load_model(model_path=None, model_name=STT_MODEL_NAME, lang=STT_LANG)
    text = recognize_from_mic(model, device=device, single_utterance=True)
    return (text or "").strip()


def ask_and_classify(
    prompt_wav: Union[str, Path],
    user_wav: Optional[Union[str, Path]],
    classifier: IntentClassifier,
    play_audio: bool,
    sim: bool,
    device: Optional[int] = None,
) -> Tuple[Intent, float, str]:
    speak_robot(prompt_wav, play_audio=play_audio)
    if sim:
        if user_wav is None:
            raise ValueError("user_wav is required in simulation mode.")
        text = transcribe_wav(user_wav)
    else:
        text = transcribe_mic(device=device)
    if not text:
        return Intent.OTHER, 0.0, ""
    intent, confidence = classifier.predict(text)
    return intent, confidence, text


def decide_two_rounds(
    classifier: IntentClassifier,
    first_user_wav: Optional[Union[str, Path]],
    second_user_wav: Optional[Union[str, Path]],
    play_audio: bool,
    sim: bool,
    device: Optional[int] = None,
) -> DecisionOutcome:
    intent1, score1, text1 = ask_and_classify(
        prompt_wav=ROBOT_PROMPT_INITIAL_WAV,
        user_wav=first_user_wav,
        classifier=classifier,
        play_audio=play_audio,
        sim=sim,
        device=device,
    )
    print(f"[Round 1] text='{text1}' | intent={intent1.value} | confidence={score1:.4f}")

    if intent1 == Intent.AFFIRMATIVE:
        print("[Action] Robot gives the rubbish bin to user.")
        speak_robot(ROBOT_ACTION_PROCEED_WAV, play_audio=play_audio)
        return DecisionOutcome.PROCEED

    if intent1 == Intent.NEGATIVE:
        print("[Action] Robot leaves.")
        speak_robot(ROBOT_ACTION_DECLINE_WAV, play_audio=play_audio)
        return DecisionOutcome.DECLINE

    intent2, score2, text2 = ask_and_classify(
        prompt_wav=ROBOT_PROMPT_CLARIFICATION_WAV,
        user_wav=second_user_wav,
        classifier=classifier,
        play_audio=play_audio,
        sim=sim,
        device=device,
    )
    print(f"[Round 2] text='{text2}' | intent={intent2.value} | confidence={score2:.4f}")

    if intent2 == Intent.AFFIRMATIVE:
        print("[Action] Robot gives the rubbish bin to user.")
        speak_robot(ROBOT_ACTION_PROCEED_WAV, play_audio=play_audio)
        return DecisionOutcome.PROCEED

    print("[Action] Robot leaves.")
    speak_robot(ROBOT_ACTION_DECLINE_WAV, play_audio=play_audio)
    return DecisionOutcome.DECLINE


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Run dialogue interface pipeline.")
    parser.add_argument("--sim", action="store_true", help="Use WAV files to simulate user input.")
    parser.add_argument("--first-user-wav", type=str, default=None, help="User response WAV path for round 1 (sim mode).")
    parser.add_argument("--second-user-wav", type=str, default=None, help="User response WAV path for round 2 (sim mode).")
    parser.add_argument(
        "--nlu-model",
        type=str,
        default=str(project_root / "models" / "nlu_intent.bin"),
        help="Path to trained NLU model.",
    )
    parser.add_argument("--no-play", action="store_true", help="Skip playing prompt WAV files.")
    parser.add_argument("--device", type=int, default=None, help="Audio input device index (mic mode only).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.sim and (not args.first_user_wav or not args.second_user_wav):
        raise ValueError("--first-user-wav and --second-user-wav are required when --sim is enabled.")
    classifier = IntentClassifier.load(args.nlu_model)
    outcome = decide_two_rounds(
        classifier=classifier,
        first_user_wav=args.first_user_wav,
        second_user_wav=args.second_user_wav,
        play_audio=not args.no_play,
        sim=args.sim,
        device=args.device,
    )
    print(f"[Final] outcome={outcome.value}")


if __name__ == "__main__":
    main()

