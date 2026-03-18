"""Generate extra robot voice prompts and record their content in content.json."""

import json
from pathlib import Path

from src.utils.text_to_speech import generate

VOICE_DIR = Path("voice_data")
MODEL = "en/en_GB/vctk/medium/en_GB-vctk-medium.onnx.json"

PROMPTS = {
    "robot_prompt": [
        ("a", "Hello! Do you have any rubbish you need to throw away?"),
        ("b", "Hi! Would you like to dispose of any waste?"),
        ("c", "Excuse me, do you have any rubbish you'd like to throw away?"),
        ("d", "Hi there! Could I take any waste off your hands?"),
    ],
    "robot_leave": [
        ("a", "Alright, no worries. Have a great day!"),
        ("b", "Okay! I will leave you to it. Goodbye!"),
        ("c", "No problem at all! Have a lovely day. Goodbye!"),
        ("d", "That's alright. I'll be on my way. Take care!"),
    ],
    "robot_proceed": [
        ("a", "Got it! Here is the bin. Please put the rubbish on the top of the bin."),
        ("b", "No problem! I will hold the rubbish bin for you. Please put the rubbish on the top of the bin."),
        ("c", "Okay! Here is the bin. Please put the rubbish on the top of the bin."),
        ("d", "Of course! I will hold the bin open for you. Please put the rubbish on the top of the bin."),
    ],
    "robot_repeat": [
        ("a", "Sorry, I didn't quite catch that. Do you have any rubbish to throw away?"),
        ("b", "Pardon? Could you please tell me if you have any waste to dispose of?"),
        ("c", "I'm sorry, I didn't understand. Do you need to throw anything away?"),
        ("d", "Could you say that again? I just want to know if you have any rubbish for me."),
    ],
}

TTS_CONFIG = {
    "volume": 1.5,
    "speed": 1.2,
    "noise_scale": 0.333,
    "noise_w_scale": 0.4,
    "use_cuda": False,
}


def load_content(path: Path) -> dict:
    if path.exists() and path.stat().st_size > 0:
        return json.loads(path.read_text())
    return {}


def main() -> None:
    content_path = VOICE_DIR / "content.json"
    content = load_content(content_path)

    for category, variants in PROMPTS.items():
        if category not in content:
            content[category] = {}
        for label, text in variants:
            filename = f"{category}_{label}.wav"
            output_path = VOICE_DIR / filename
            print(f"[TTS] Generating {filename}: \"{text}\"")
            generate(
                model_path=MODEL,
                text=text,
                output_path=str(output_path),
                **TTS_CONFIG,
            )
            content[category][label] = text
            print(f"      -> saved to {output_path}")

    content_path.write_text(json.dumps(content, indent=2, ensure_ascii=False))
    print(f"\n[Done] content.json updated at {content_path}")


if __name__ == "__main__":
    main()
