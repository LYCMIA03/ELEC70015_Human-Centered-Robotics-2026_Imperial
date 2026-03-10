"""Dialogue pipeline with rule-based decision manager."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from pathlib import Path
from typing import Any, Optional, Tuple, Union

from vosk import Model as SttModel

from src.utils.audio_playback import speak_robot
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


def _intent_label(intent: Any) -> str:
    """Normalize intent objects from different backends to a lowercase label."""
    if hasattr(intent, "value"):
        return str(intent.value).strip().lower()
    return str(intent).strip().lower()


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
    classifier: Any,
    stt_model: SttModel,
    play_audio: bool,
    sim: bool,
    device: Optional[int] = None,
) -> Tuple[str, float, str]:
    speak_robot(prompt_wav, play_audio=play_audio)
    if sim:
        if user_wav is None:
            raise ValueError("user_wav is required in simulation mode.")
        text = transcribe_wav(user_wav, stt_model)
    else:
        text = transcribe_mic(stt_model, device=device)
    if not text:
        return "other", 0.0, ""
    intent, confidence = classifier.predict(text)
    return _intent_label(intent), confidence, text


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
    classifier: Any,
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
        intent_label, score, text = ask_and_classify(
            prompt_wav=prompt_wav,
            user_wav=user_wav,
            classifier=classifier,
            stt_model=stt_model,
            play_audio=play_audio,
            sim=sim,
            device=device,
        )
        print(f"[Round {idx}] text='{text}' | intent={intent_label} | confidence={score:.4f}")
        if intent_label == "affirmative":
            return _execute_decision(DecisionOutcome.PROCEED, play_audio)
        if intent_label == "negative":
            return _execute_decision(DecisionOutcome.DECLINE, play_audio)
    return _execute_decision(DecisionOutcome.DECLINE, play_audio)


def _load_classifier(backend: str, nlu_path: str):
    if backend == "bert":
        from src.utils.nlu_intent_bert import IntentClassifierBert
        return IntentClassifierBert.load(nlu_path)
    if backend == "fasttext":
        from src.utils.nlu_intent import IntentClassifier
        return IntentClassifier.load(nlu_path)
    raise ValueError(f"Unknown backend: {backend}")


def _resolve_backend_and_model(project_root: Path, backend: str, model_arg: Optional[str]) -> Tuple[str, str]:
    bert_path = project_root / "models" / "nlu_intent_bert"
    fasttext_path = project_root / "models" / "nlu_intent.bin"

    if model_arg:
        model_path = Path(model_arg)
    elif backend == "bert":
        model_path = bert_path
    elif backend == "fasttext":
        model_path = fasttext_path
    else:
        model_path = bert_path if bert_path.exists() else fasttext_path

    selected_backend = backend
    if backend == "auto":
        selected_backend = "bert" if model_path.is_dir() else "fasttext"
    return selected_backend, str(model_path)


def _load_models_parallel(backend: str, nlu_path: str) -> Tuple[Any, SttModel]:
    """Load NLU and STT models concurrently to reduce startup time."""
    with ThreadPoolExecutor(max_workers=2) as pool:
        nlu_future = pool.submit(_load_classifier, backend, nlu_path)
        stt_future = pool.submit(load_model, None, STT_MODEL_NAME, STT_LANG)
        return nlu_future.result(), stt_future.result()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run dialogue interface pipeline.")
    parser.add_argument("--sim", action="store_true", help="Use WAV files to simulate user input.")
    parser.add_argument("--first-user-wav", type=str, default=None, help="User response WAV path for round 1 (sim mode).")
    parser.add_argument("--second-user-wav", type=str, default=None, help="User response WAV path for round 2 (sim mode).")
    parser.add_argument("--nlu-model", type=str, default=None, help="Path to trained NLU model.")
    parser.add_argument(
        "--nlu-backend",
        type=str,
        default="auto",
        choices=("auto", "bert", "fasttext"),
        help="Select NLU backend. auto prefers BERT model dir, otherwise FastText.",
    )
    parser.add_argument("--no-play", action="store_true", help="Skip playing prompt WAV files.")
    parser.add_argument("--device", type=int, default=None, help="Audio input device index (mic mode only).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.sim and (not args.first_user_wav or not args.second_user_wav):
        raise ValueError("--first-user-wav and --second-user-wav are required when --sim is enabled.")

    project_root = Path(__file__).resolve().parent.parent
    backend, model_path = _resolve_backend_and_model(project_root, args.nlu_backend, args.nlu_model)

    print("[Init] Loading models...")
    print(f"[Init] NLU backend={backend}, model={model_path}")
    try:
        classifier, stt_model = _load_models_parallel(backend, model_path)
    except Exception as exc:
        if args.nlu_backend == "auto" and backend == "bert":
            fallback_model = str(project_root / "models" / "nlu_intent.bin")
            print(f"[Init] BERT load failed ({exc}); fallback to fasttext model={fallback_model}")
            backend = "fasttext"
            model_path = fallback_model
            classifier, stt_model = _load_models_parallel(backend, model_path)
        else:
            raise
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
