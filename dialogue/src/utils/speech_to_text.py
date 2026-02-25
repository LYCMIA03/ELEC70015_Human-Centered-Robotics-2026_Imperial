"""Speech-to-text using Vosk offline recognition.

Usage examples:
    # Recognize from a WAV file
    python -m src.utils.speech_to_text --wav rubbish_question.wav

    # Recognize from microphone (real-time)
    python -m src.utils.speech_to_text --mic
"""

import argparse
import json
import queue
import wave
from pathlib import Path
from typing import Optional

from vosk import KaldiRecognizer, Model, SetLogLevel


def recognize_from_wav(
    wav_path: str,
    model: Model,
) -> str:
    """Recognize speech from a WAV file."""
    wf = wave.open(wav_path, "rb")
    if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
        raise ValueError("WAV file must be mono 16-bit PCM format.")

    rec = KaldiRecognizer(model, wf.getframerate())
    rec.SetWords(True)

    results = []
    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            partial = json.loads(rec.Result())
            if partial.get("text"):
                results.append(partial["text"])

    final = json.loads(rec.FinalResult())
    if final.get("text"):
        results.append(final["text"])

    wf.close()
    return " ".join(results)


def recognize_from_mic(
    model: Model,
    device: Optional[int] = None,
    sample_rate: Optional[int] = None,
    single_utterance: bool = False,
) -> Optional[str]:
    """Recognize speech from microphone in real-time or one utterance."""
    import sounddevice as sd

    q: queue.Queue[bytes] = queue.Queue()

    def audio_callback(indata, frames, time, status):
        if status:
            print(f"[Audio warning] {status}")
        q.put(bytes(indata))

    if sample_rate is None:
        device_info = sd.query_devices(device, "input")
        sample_rate = int(device_info["default_samplerate"])

    rec = KaldiRecognizer(model, sample_rate)

    print("=" * 60)
    if single_utterance:
        print("Listening for one utterance...")
    else:
        print("Listening... (Press Ctrl+C to stop)")
    print("=" * 60)

    try:
        with sd.RawInputStream(
            samplerate=sample_rate,
            blocksize=8000,
            device=device,
            dtype="int16",
            channels=1,
            callback=audio_callback,
        ):
            while True:
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip()
                    if text:
                        if single_utterance:
                            return text
                        print(f">> {text}")
                else:
                    partial = json.loads(rec.PartialResult())
                    partial_text = partial.get("partial", "")
                    if partial_text:
                        print(f"   ... {partial_text}", end="\r")
    except KeyboardInterrupt:
        print("\nStopped.")
    return None

def load_model(
    model_path: Optional[str] = None,
    model_name: str = "vosk-model-small-en-us-0.15",
    lang: str = "en-us",
) -> Model:
    """Load a Vosk model.

    Priority:
    1) If `model_path` exists, use it.
    2) Otherwise use (`model_name`, `lang`) and let Vosk auto-download/cache.
    """
    if model_path and Path(model_path).exists():
        return Model(model_path=model_path)
    try:
        return Model(model_name=model_name, lang=lang)
    except Exception as exc:
        raise RuntimeError(
            f"Failed to load/download Vosk model '{model_name}' (lang={lang})."
        ) from exc


def list_input_devices() -> None:
    """Print all available audio input devices."""
    import sounddevice as sd
    devices = sd.query_devices()
    print(f"{'Index':<6} {'Name':<45} {'Channels':<10} {'Sample Rate'}")
    print("-" * 75)
    for i, d in enumerate(devices):
        if d["max_input_channels"] > 0:
            print(f"[{i:<4}] {d['name']:<45} {d['max_input_channels']:<10} {int(d['default_samplerate'])} Hz")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Speech-to-text using Vosk offline recognition."
    )

    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--wav", type=str, help="Path to a WAV file to recognize.")
    source.add_argument("--mic", action="store_true", help="Recognize from microphone in real-time.")
    source.add_argument("--list-devices", action="store_true", help="List all available audio input devices and exit.")

    parser.add_argument("--model", type=str, default=None, help="Path to a local Vosk model directory.")
    parser.add_argument(
        "--model-name",
        type=str,
        default="vosk-model-small-en-us-0.15",
        help="Vosk model name for auto-download.",
    )
    parser.add_argument("--lang", type=str, default="en-us", help="Language code for auto-download model.")
    parser.add_argument("--device", type=int, default=None, help="Audio input device index (for --mic mode).")
    parser.add_argument("--sample-rate", type=int, default=None, help="Sample rate in Hz (for --mic mode).")
    parser.add_argument("--quiet", action="store_true", help="Suppress Vosk debug logs.")

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_devices:
        list_input_devices()
        return

    if args.quiet:
        SetLogLevel(-1)

    model = load_model(
        model_path=args.model,
        model_name=args.model_name,
        lang=args.lang,
    )

    if args.wav:
        text = recognize_from_wav(args.wav, model)
        print(f"Recognized: {text}")
    else:
        recognize_from_mic(model, device=args.device, sample_rate=args.sample_rate)


if __name__ == "__main__":
    main()

