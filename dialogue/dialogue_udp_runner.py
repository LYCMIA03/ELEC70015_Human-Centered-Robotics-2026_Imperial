#!/usr/bin/env python3
"""
Host-side dialogue runner:
- listen for navigation_success trigger via UDP
- run dialogue decision pipeline (mic or sim)
- send trash_action (decline=0, proceed=1) back via UDP
"""
import argparse
import json
import socket
import sys
import time
import traceback
from pathlib import Path


def _parse_boolish(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "y", "proceed"):
            return True
        if text in ("0", "false", "no", "n", "decline", "terminate"):
            return False
    return None


def _drain_socket(sock):
    """Discard all buffered UDP packets after a dialogue round."""
    sock.setblocking(False)
    try:
        while True:
            try:
                sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                break
    finally:
        sock.setblocking(True)
        sock.settimeout(0.2)


def _extract_navigation_success(payload):
    if isinstance(payload, dict):
        for key in ("navigation_success", "value", "success", "result"):
            if key in payload:
                return _parse_boolish(payload[key])
        return None
    return _parse_boolish(payload)


def parse_args():
    default_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Run dialogue after navigation_success trigger and return trash_action.")
    parser.add_argument("--dialogue-root", default=str(default_root), help="Path to dialogue module root.")
    parser.add_argument("--listen-host", default="0.0.0.0", help="UDP listen host for navigation_success trigger.")
    parser.add_argument("--listen-port", type=int, default=16041, help="UDP listen port for navigation_success trigger.")
    parser.add_argument("--send-host", default="127.0.0.1", help="UDP destination host for trash_action.")
    parser.add_argument("--send-port", type=int, default=16032, help="UDP destination port for trash_action.")
    parser.add_argument(
        "--nlu-model",
        default=None,
        help="NLU model path. Default auto-order: <dialogue-root>/models/nlu_intent_bert then nlu_intent.bin",
    )
    parser.add_argument(
        "--nlu-backend",
        default="auto",
        choices=("auto", "bert", "fasttext"),
        help="NLU backend. auto: try BERT first then FastText fallback.",
    )
    parser.add_argument("--device", type=int, default=None, help="Microphone input device index.")
    parser.add_argument("--no-play", action="store_true", help="Skip playing robot wav prompts.")
    parser.add_argument("--sim", action="store_true", help="Use wav files instead of microphone input.")
    parser.add_argument("--first-user-wav", type=str, default=None, help="Round-1 user wav in sim mode.")
    parser.add_argument("--second-user-wav", type=str, default=None, help="Round-2 user wav in sim mode.")
    parser.add_argument("--cooldown-sec", type=float, default=1.0, help="Minimum interval between triggers.")
    return parser.parse_args()


def _resolve_user_wav_arg(wav_arg, dialogue_root, launch_cwd):
    if not wav_arg:
        return wav_arg
    p = Path(wav_arg)
    if p.is_absolute():
        return str(p)

    # 1) Relative to launch cwd (common when launched from repo root).
    c1 = (launch_cwd / p).resolve()
    if c1.is_file():
        return str(c1)

    # 2) Relative to dialogue root (common when passing voice_data/...).
    c2 = (dialogue_root / p).resolve()
    if c2.is_file():
        return str(c2)

    return wav_arg


def _resolve_fasttext_model_path(path: Path) -> Path:
    if path.is_dir():
        for name in ("nlu_intent.bin", "nlu_intent_fast_text.bin"):
            candidate = path / name
            if candidate.is_file():
                return candidate
        raise FileNotFoundError(f"No FastText model found under: {path}")
    return path


def _load_nlu_classifier(dialogue_root: Path, backend: str, model_arg):
    bert_default = dialogue_root / "models" / "nlu_intent_bert"
    fasttext_default = dialogue_root / "models" / "nlu_intent.bin"

    def _load_bert(path: Path):
        if not path.exists():
            raise FileNotFoundError(f"BERT model path not found: {path}")
        from src.utils.nlu_intent_bert import IntentClassifierBert
        return IntentClassifierBert.load(str(path))

    def _load_fasttext(path: Path):
        from src.utils.nlu_intent import IntentClassifier
        resolved = _resolve_fasttext_model_path(path)
        if not resolved.is_file():
            raise FileNotFoundError(f"FastText model not found: {resolved}")
        return IntentClassifier.load(str(resolved))

    if backend == "bert":
        chosen = Path(model_arg).resolve() if model_arg else bert_default
        return _load_bert(chosen), "bert", chosen

    if backend == "fasttext":
        chosen = Path(model_arg).resolve() if model_arg else fasttext_default
        return _load_fasttext(chosen), "fasttext", _resolve_fasttext_model_path(chosen)

    if model_arg:
        model_path = Path(model_arg).resolve()
        candidates = [("bert", model_path), ("fasttext", model_path)]
    else:
        candidates = [("bert", bert_default), ("fasttext", fasttext_default)]

    errors = []
    for candidate_backend, candidate_path in candidates:
        try:
            if candidate_backend == "bert":
                return _load_bert(candidate_path), "bert", candidate_path
            model_file = _resolve_fasttext_model_path(candidate_path)
            return _load_fasttext(candidate_path), "fasttext", model_file
        except Exception as exc:
            errors.append(f"{candidate_backend}@{candidate_path}: {exc}")
            continue

    raise RuntimeError("Failed to load NLU classifier. " + " | ".join(errors))


def main():
    args = parse_args()
    launch_cwd = Path.cwd()
    dialogue_root = Path(args.dialogue_root).resolve()
    if not dialogue_root.is_dir():
        raise FileNotFoundError(f"dialogue root not found: {dialogue_root}")

    if args.sim and (not args.first_user_wav or not args.second_user_wav):
        raise ValueError("--first-user-wav and --second-user-wav are required when --sim is enabled.")

    # dialogue_manager.py uses relative voice_data paths; keep cwd at dialogue root.
    sys.path.insert(0, str(dialogue_root))
    import os

    os.chdir(dialogue_root)

    args.first_user_wav = _resolve_user_wav_arg(args.first_user_wav, dialogue_root, launch_cwd)
    args.second_user_wav = _resolve_user_wav_arg(args.second_user_wav, dialogue_root, launch_cwd)

    from src.dialogue_manager import STT_LANG, STT_MODEL_NAME, decide_two_rounds
    from src.utils.speech_to_text import load_model

    print("[Init] Loading dialogue models...", flush=True)
    classifier, backend_used, model_used = _load_nlu_classifier(
        dialogue_root=dialogue_root,
        backend=args.nlu_backend,
        model_arg=args.nlu_model,
    )
    print(f"[Init] NLU backend={backend_used}, model={model_used}", flush=True)
    stt_model = load_model(model_path=None, model_name=STT_MODEL_NAME, lang=STT_LANG)
    print("[Init] Dialogue runner ready.", flush=True)

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind((args.listen_host, args.listen_port))
    recv_sock.settimeout(0.2)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[UDP] Listening trigger on {args.listen_host}:{args.listen_port}", flush=True)
    print(f"[UDP] Sending trash_action to {args.send_host}:{args.send_port}", flush=True)

    last_trigger_t = 0.0
    while True:
        try:
            data, addr = recv_sock.recvfrom(65535)
        except socket.timeout:
            continue

        now = time.time()
        if now - last_trigger_t < max(0.0, args.cooldown_sec):
            continue

        try:
            text = data.decode("utf-8").strip()
            payload = json.loads(text) if text.startswith("{") else text
            nav_ok = _extract_navigation_success(payload)
        except Exception:
            nav_ok = None

        if nav_ok is not True:
            continue

        last_trigger_t = now
        print(f"[Trigger] navigation_success=1 from {addr}, starting dialogue...", flush=True)
        try:
            outcome = decide_two_rounds(
                classifier=classifier,
                stt_model=stt_model,
                first_user_wav=args.first_user_wav,
                second_user_wav=args.second_user_wav,
                play_audio=not args.no_play,
                sim=args.sim,
                device=args.device,
            )
        except Exception as exc:
            print(f"[Error] dialogue round failed: {exc}", flush=True)
            traceback.print_exc()
            _drain_socket(recv_sock)
            last_trigger_t = time.time()
            continue

        action = 1 if outcome.value == "proceed" else 0
        result_payload = {
            "stamp": time.time(),
            "trash_action": action,
            "decision": outcome.value,
            "source": "dialogue_udp_runner",
        }
        try:
            send_sock.sendto(
                json.dumps(result_payload, separators=(",", ":")).encode("utf-8"),
                (args.send_host, args.send_port),
            )
            print(f"[Result] outcome={outcome.value} -> trash_action={action}", flush=True)
        except Exception as exc:
            print(f"[Error] failed to send trash_action UDP: {exc}", flush=True)
            traceback.print_exc()

        _drain_socket(recv_sock)
        last_trigger_t = time.time()


if __name__ == "__main__":
    main()
