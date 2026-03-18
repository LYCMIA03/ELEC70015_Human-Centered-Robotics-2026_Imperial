"""Generate speech audio from text using Piper TTS."""

import argparse
import urllib.request
import wave
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
PIPER_HF_BASE = (
    "https://huggingface.co/rhasspy/piper-voices/resolve/main/"
)


def _download_to(url: str, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url) as resp, open(dst, "wb") as f:
        f.write(resp.read())


def ensure_tts_model(model_ref: str) -> Path:
    """Ensure Piper TTS model files (.onnx + .onnx.json) exist locally.

    model_ref can be:
      - A HuggingFace sub-path, e.g.
        "en/en_GB/vctk/medium/en_GB-vctk-medium.onnx.json"
      - A local .onnx file path that already exists on disk.
    """
    candidate = Path(model_ref)
    if candidate.suffix == ".onnx" and candidate.exists():
        return candidate

    if not model_ref.endswith((".onnx", ".onnx.json")):
        raise ValueError(
            f"model_ref should end with .onnx or .onnx.json, got: {model_ref}"
        )

    hf_json_sub = model_ref if model_ref.endswith(".onnx.json") else model_ref + ".json"
    hf_onnx_sub = hf_json_sub.removesuffix(".json")

    json_name = Path(hf_json_sub).name
    onnx_name = Path(hf_onnx_sub).name
    onnx_path = MODELS_DIR / onnx_name
    json_path = MODELS_DIR / json_name

    if not onnx_path.exists():
        print(f"[TTS] Downloading {onnx_name} ...")
        _download_to(PIPER_HF_BASE + hf_onnx_sub, onnx_path)
    if not json_path.exists():
        print(f"[TTS] Downloading {json_name} ...")
        _download_to(PIPER_HF_BASE + hf_json_sub, json_path)

    return onnx_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate speech audio from text using Piper TTS."
    )
    parser.add_argument("--model", type=str, default=None, help="Path to the Piper voice model (.onnx file).")
    parser.add_argument("--text", type=str, default=None, help="Text to synthesize into speech.")
    parser.add_argument("--output", type=str, default=None, help="Output WAV file path.")
    parser.add_argument("--volume", type=float, default=None, help="Volume scale.")
    parser.add_argument("--speed", type=float, default=None, help="Speed scale (length_scale).")
    parser.add_argument("--noise-scale", type=float, default=None, help="Audio variation.")
    parser.add_argument("--noise-w-scale", type=float, default=None, help="Speaking variation.")
    parser.add_argument("--cuda", dest="cuda", action="store_true", help="Use CUDA for GPU acceleration.")
    parser.add_argument("--no-cuda", dest="cuda", action="store_false", help="Disable CUDA.")
    parser.set_defaults(cuda=None)
    return parser.parse_args()


def generate(
    model_path: str,
    text: str,
    output_path: str = "output.wav",
    volume: float = 1.0,
    speed: float = 1.0,
    noise_scale: float = 0.667,
    noise_w_scale: float = 0.8,
    use_cuda: bool = False,
) -> Path:
    """Synthesize text to a WAV file and return the output path."""
    from piper.config import SynthesisConfig
    from piper.voice import PiperVoice

    model_file = ensure_tts_model(model_path)

    voice = PiperVoice.load(str(model_file), use_cuda=use_cuda)

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    syn_config = SynthesisConfig(
        volume=volume,
        length_scale=speed,
        noise_scale=noise_scale,
        noise_w_scale=noise_w_scale,
    )

    with wave.open(str(out), "wb") as wav_file:
        voice.synthesize_wav(text, wav_file, syn_config=syn_config)

    return out


def main() -> None:
    # You can edit these defaults directly for quick local generation.
    config = {
        "model_path": "en/en_GB/vctk/medium/en_GB-vctk-medium.onnx.json",
        "text": "You are so stupid!",
        "output_path": "voice_data/sim_user_answer_other_b.wav",
        "volume": 1.0,
        "speed": 2.0,
        "noise_scale": 0.333,
        "noise_w_scale": 0.4,
        "use_cuda": False,
    }

    # Keep CLI support: provided args override defaults above.
    args = parse_args()
    if args.model is not None:
        config["model_path"] = args.model
    if args.text is not None:
        config["text"] = args.text
    if args.output is not None:
        config["output_path"] = args.output
    if args.volume is not None:
        config["volume"] = args.volume
    if args.speed is not None:
        config["speed"] = args.speed
    if args.noise_scale is not None:
        config["noise_scale"] = args.noise_scale
    if args.noise_w_scale is not None:
        config["noise_w_scale"] = args.noise_w_scale
    if args.cuda is not None:
        config["use_cuda"] = args.cuda

    out = generate(
        model_path=config["model_path"],
        text=config["text"],
        output_path=config["output_path"],
        volume=config["volume"],
        speed=config["speed"],
        noise_scale=config["noise_scale"],
        noise_w_scale=config["noise_w_scale"],
        use_cuda=config["use_cuda"],
    )
    print(f"Audio saved to: {out.resolve()}")


if __name__ == "__main__":
    main()

