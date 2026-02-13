"""Dialogue pipeline with rule-based decision manager."""

import argparse
import glob
import random
import subprocess
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple, Union

from src.utils.nlu_intent import Intent, IntentClassifier
from src.utils.speech_to_text import load_model, recognize_from_wav

ROBOT_PROMPT_INITIAL_WAV = "voice_data/robot_voice_library_prompt_throw_rubbish_*.wav"
ROBOT_PROMPT_CLARIFICATION_WAV = "voice_data/robot_voice_library_prompt_throw_rubbish_*.wav"
ROBOT_ACTION_PROCEED_WAV = "voice_data/robot_voice_library_answer_affirmative_*.wav"
ROBOT_ACTION_DECLINE_WAV = "voice_data/robot_voice_library_answer_negative_*.wav"


class DecisionOutcome(str, Enum):
    """Final interaction outcomes."""

    PROCEED = "proceed"
    DECLINE = "decline"
    TERMINATE = "terminate"


def _resolve_wav_candidates(wav_spec: Union[str, Path]) -> List[Path]:
    """Resolve a wav spec into candidate files.

    Supports:
    - exact file path
    - glob pattern like voice_data/robot_leave_*.wav
    """
    spec = str(wav_spec)
    base = Path(spec)
    if "*" not in spec and "?" not in spec and "[" not in spec:
        return [base]

    return [Path(p) for p in sorted(glob.glob(spec)) if Path(p).is_file()]


def sample_wav(wav_spec: Union[str, Path]) -> Path:
    """Sample one WAV file from path or glob pattern."""
    candidates = _resolve_wav_candidates(wav_spec)
    if not candidates:
        raise FileNotFoundError(f"No WAV file matched: {wav_spec}")
    return random.choice(candidates)


def play_wav(wav_path: Union[str, Path]) -> None:
    """Play a wav file with available system player."""
    wav = sample_wav(wav_path)
    if not wav.exists():
        raise FileNotFoundError(f"WAV file not found: {wav}")

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


def transcribe_wav(
    wav_path: Union[str, Path],
    stt_model_path: Optional[str],
    stt_model_name: str,
    stt_lang: str,
) -> str:
    """Run STT on a WAV file and return recognized text."""
    selected_wav = sample_wav(wav_path)
    model = load_model(model_path=stt_model_path, model_name=stt_model_name, lang=stt_lang)
    return recognize_from_wav(str(selected_wav), model).strip()


def ask_and_classify(
    prompt_wav: Union[str, Path],
    user_wav: Union[str, Path],
    classifier: IntentClassifier,
    stt_model_path: Optional[str],
    stt_model_name: str,
    stt_lang: str,
    play_audio: bool,
) -> Tuple[Intent, float, str]:
    """Play prompt, transcribe user reply, and classify intent."""
    if play_audio:
        play_wav(prompt_wav)
    text = transcribe_wav(
        wav_path=user_wav,
        stt_model_path=stt_model_path,
        stt_model_name=stt_model_name,
        stt_lang=stt_lang,
    )
    if not text:
        return Intent.OTHER, 0.0, ""
    intent, confidence = classifier.predict(text)
    return intent, confidence, text


def decide_two_rounds(
    classifier: IntentClassifier,
    first_user_wav: Union[str, Path],
    second_user_wav: Union[str, Path],
    stt_model_path: Optional[str],
    stt_model_name: str,
    stt_lang: str,
    play_audio: bool,
) -> DecisionOutcome:
    """Apply the 2-round rule-based decision policy."""
    intent1, score1, text1 = ask_and_classify(
        prompt_wav=ROBOT_PROMPT_INITIAL_WAV,
        user_wav=first_user_wav,
        classifier=classifier,
        stt_model_path=stt_model_path,
        stt_model_name=stt_model_name,
        stt_lang=stt_lang,
        play_audio=play_audio,
    )
    print(f"[Round 1] text='{text1}' | intent={intent1.value} | confidence={score1:.4f}")

    if intent1 == Intent.AFFIRMATIVE:
        print("[Action] Robot gives the rubbish bin to user.")
        if play_audio:
            play_wav(ROBOT_ACTION_PROCEED_WAV)
        return DecisionOutcome.PROCEED

    if intent1 == Intent.NEGATIVE:
        print("[Action] Robot leaves.")
        if play_audio:
            play_wav(ROBOT_ACTION_DECLINE_WAV)
        return DecisionOutcome.DECLINE

    intent2, score2, text2 = ask_and_classify(
        prompt_wav=ROBOT_PROMPT_CLARIFICATION_WAV,
        user_wav=second_user_wav,
        classifier=classifier,
        stt_model_path=stt_model_path,
        stt_model_name=stt_model_name,
        stt_lang=stt_lang,
        play_audio=play_audio,
    )
    print(f"[Round 2] text='{text2}' | intent={intent2.value} | confidence={score2:.4f}")

    if intent2 == Intent.AFFIRMATIVE:
        print("[Action] Robot gives the rubbish bin to user.")
        if play_audio:
            play_wav(ROBOT_ACTION_PROCEED_WAV)
        return DecisionOutcome.PROCEED

    print("[Action] Robot leaves.")
    if play_audio:
        play_wav(ROBOT_ACTION_DECLINE_WAV)
    return DecisionOutcome.DECLINE


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Run dialogue interface pipeline.")
    parser.add_argument("--first-user-wav", type=str, required=True, help="User response WAV path for round 1.")
    parser.add_argument("--second-user-wav", type=str, required=True, help="User response WAV path for round 2.")
    parser.add_argument(
        "--nlu-model",
        type=str,
        default=str(project_root / "models" / "nlu_intent.bin"),
        help="Path to trained NLU model.",
    )
    parser.add_argument("--stt-model", type=str, default=None, help="Optional local Vosk model directory.")
    parser.add_argument("--stt-model-name", type=str, default="vosk-model-small-en-us-0.15")
    parser.add_argument("--stt-lang", type=str, default="en-us")
    parser.add_argument("--no-play", action="store_true", help="Skip playing prompt WAV files.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    classifier = IntentClassifier.load(args.nlu_model)
    outcome = decide_two_rounds(
        classifier=classifier,
        first_user_wav=args.first_user_wav,
        second_user_wav=args.second_user_wav,
        stt_model_path=args.stt_model,
        stt_model_name=args.stt_model_name,
        stt_lang=args.stt_lang,
        play_audio=not args.no_play,
    )
    print(f"[Final] outcome={outcome.value}")


if __name__ == "__main__":
    main()

