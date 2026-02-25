"""Generate extra robot voice prompts and record their content in content.json."""

import json
from pathlib import Path

from src.utils.text_to_speech import generate

VOICE_DIR = Path("voice_data")
MODEL = "en_GB-southern_english_female-low"

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
        ("a", "Sure! Here is the bin. Please go ahead and throw it away."),
        ("b", "Great! I will hold the rubbish bin for you. Please go ahead."),
        ("c", "Wonderful! Here is the rubbish bin. Please go ahead."),
        ("d", "Of course! I will hold the bin open for you."),
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
    "noise_scale": 0.667,
    "noise_w_scale": 0.8,
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
