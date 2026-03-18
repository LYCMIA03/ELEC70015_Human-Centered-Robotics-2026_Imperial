"""Generate speech audio from text using Piper TTS."""

import argparse
import urllib.request
import wave
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
DEFAULT_TTS_MODEL_NAME = "en_GB-southern_english_female-low"
DEFAULT_TTS_ONNX_PATH = MODELS_DIR / f"{DEFAULT_TTS_MODEL_NAME}.onnx"
DEFAULT_TTS_JSON_PATH = MODELS_DIR / f"{DEFAULT_TTS_MODEL_NAME}.onnx.json"
DEFAULT_TTS_ONNX_URL = (
    "https://huggingface.co/rhasspy/piper-voices/resolve/main/"
    "en/en_GB/southern_english_female/low/en_GB-southern_english_female-low.onnx"
)
DEFAULT_TTS_JSON_URL = (
    "https://huggingface.co/rhasspy/piper-voices/resolve/main/"
    "en/en_GB/southern_english_female/low/en_GB-southern_english_female-low.onnx.json"
)


def _download_to(url: str, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url) as resp, open(dst, "wb") as f:
        f.write(resp.read())


def ensure_tts_model(model_ref: str) -> Path:
    """Ensure default TTS model files exist in models/ and return onnx path."""
    allowed_refs = {
        DEFAULT_TTS_MODEL_NAME,
        DEFAULT_TTS_ONNX_PATH.name,
        str(DEFAULT_TTS_ONNX_PATH),
    }
    if model_ref not in allowed_refs:
        raise ValueError(
            f"Only '{DEFAULT_TTS_MODEL_NAME}' is supported. Got: {model_ref}"
        )

    if not DEFAULT_TTS_ONNX_PATH.exists():
        _download_to(DEFAULT_TTS_ONNX_URL, DEFAULT_TTS_ONNX_PATH)
    if not DEFAULT_TTS_JSON_PATH.exists():
        _download_to(DEFAULT_TTS_JSON_URL, DEFAULT_TTS_JSON_PATH)
    return DEFAULT_TTS_ONNX_PATH


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
        "model_path": "en_GB-southern_english_female-low",
        "text": "You are so stupid!",
        "output_path": "voice_data/sim_user_answer_other_b.wav",
        "volume": 1.0,
        "speed": 1.0,
        "noise_scale": 0.667,
        "noise_w_scale": 0.8,
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

