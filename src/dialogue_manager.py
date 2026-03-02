"""Dialogue pipeline with rule-based decision manager."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from pathlib import Path
from typing import Optional, Tuple, Union

from vosk import Model as SttModel

from src.utils.audio_playback import speak_robot
from src.utils.nlu_intent_bert import Intent, IntentClassifierBert
from src.utils.speech_to_text import load_model, recognize_from_mic, recognize_from_wav

ROBOT_PROMPT_INITIAL_WAV = "voice_data/robot_prompt_*.wav"
ROBOT_PROMPT_CLARIFICATION_WAV = "voice_data/robot_repeat_*.wav"
ROBOT_ACTION_PROCEED_WAV = "voice_data/robot_proceed_*.wav"
ROBOT_ACTION_DECLINE_WAV = "voice_data/robot_leave_*.wav"

# No hardcoded local model path.
STT_MODEL_NAME = "vosk-model-small-en-us-0.15"
STT_LANG = "en-us"


class DecisionOutcome(str, Enum):
    PROCEED = "proceed"
    DECLINE = "decline"
    TERMINATE = "terminate"


def transcribe_wav(wav_path: Union[str, Path], stt_model: SttModel) -> str:
    user_wav = Path(wav_path)
    if not user_wav.is_file():
        raise FileNotFoundError(f"User WAV file not found: {wav_path}")
    return recognize_from_wav(str(user_wav), stt_model).strip()


def transcribe_mic(stt_model: SttModel, device: Optional[int] = None) -> str:
    text = recognize_from_mic(stt_model, device=device, single_utterance=True)
    return (text or "").strip()


def ask_and_classify(
    prompt_wav: Union[str, Path],
    user_wav: Optional[Union[str, Path]],
    classifier: IntentClassifierBert,
    stt_model: SttModel,
    play_audio: bool,
    sim: bool,
    device: Optional[int] = None,
) -> Tuple[Intent, float, str]:
    speak_robot(prompt_wav, play_audio=play_audio)
    if sim:
        if user_wav is None:
            raise ValueError("user_wav is required in simulation mode.")
        text = transcribe_wav(user_wav, stt_model)
    else:
        text = transcribe_mic(stt_model, device=device)
    if not text:
        return Intent.OTHER, 0.0, ""
    intent, confidence = classifier.predict(text)
    return intent, confidence, text


_DECISION_ACTIONS = {
    DecisionOutcome.PROCEED: ("[Action] Robot gives the rubbish bin to user.", ROBOT_ACTION_PROCEED_WAV),
    DecisionOutcome.DECLINE: ("[Action] Robot leaves.", ROBOT_ACTION_DECLINE_WAV),
}


def _execute_decision(outcome: DecisionOutcome, play_audio: bool) -> DecisionOutcome:
    msg, wav = _DECISION_ACTIONS[outcome]
    print(msg)
    speak_robot(wav, play_audio=play_audio)
    return outcome


def decide_two_rounds(
    classifier: IntentClassifierBert,
    stt_model: SttModel,
    first_user_wav: Optional[Union[str, Path]],
    second_user_wav: Optional[Union[str, Path]],
    play_audio: bool,
    sim: bool,
    device: Optional[int] = None,
) -> DecisionOutcome:
    rounds = [
        (ROBOT_PROMPT_INITIAL_WAV, first_user_wav),
        (ROBOT_PROMPT_CLARIFICATION_WAV, second_user_wav),
    ]
    for idx, (prompt_wav, user_wav) in enumerate(rounds, 1):
        intent, score, text = ask_and_classify(
            prompt_wav=prompt_wav,
            user_wav=user_wav,
            classifier=classifier,
            stt_model=stt_model,
            play_audio=play_audio,
            sim=sim,
            device=device,
        )
        print(f"[Round {idx}] text='{text}' | intent={intent.value} | confidence={score:.4f}")
        if intent == Intent.AFFIRMATIVE:
            return _execute_decision(DecisionOutcome.PROCEED, play_audio)
        if intent == Intent.NEGATIVE:
            return _execute_decision(DecisionOutcome.DECLINE, play_audio)
    return _execute_decision(DecisionOutcome.DECLINE, play_audio)


def _load_models_parallel(nlu_path: str) -> Tuple[IntentClassifierBert, SttModel]:
    """Load NLU and STT models concurrently to halve startup time."""
    with ThreadPoolExecutor(max_workers=2) as pool:
        nlu_future = pool.submit(IntentClassifierBert.load, nlu_path)
        stt_future = pool.submit(load_model, None, STT_MODEL_NAME, STT_LANG)
        return nlu_future.result(), stt_future.result()


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Run dialogue interface pipeline.")
    parser.add_argument("--sim", action="store_true", help="Use WAV files to simulate user input.")
    parser.add_argument("--first-user-wav", type=str, default=None, help="User response WAV path for round 1 (sim mode).")
    parser.add_argument("--second-user-wav", type=str, default=None, help="User response WAV path for round 2 (sim mode).")
    parser.add_argument(
        "--nlu-model",
        type=str,
        default=str(project_root / "models" / "nlu_intent_bert"),
        help="Path to trained NLU model.",
    )
    parser.add_argument("--no-play", action="store_true", help="Skip playing prompt WAV files.")
    parser.add_argument("--device", type=int, default=None, help="Audio input device index (mic mode only).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.sim and (not args.first_user_wav or not args.second_user_wav):
        raise ValueError("--first-user-wav and --second-user-wav are required when --sim is enabled.")
    print("[Init] Loading models...")
    classifier, stt_model = _load_models_parallel(args.nlu_model)
    print("[Init] Ready.")
    outcome = decide_two_rounds(
        classifier=classifier,
        stt_model=stt_model,
        first_user_wav=args.first_user_wav,
        second_user_wav=args.second_user_wav,
        play_audio=not args.no_play,
        sim=args.sim,
        device=args.device,
    )
    print(f"[Final] outcome={outcome.value}")


if __name__ == "__main__":
    main()

