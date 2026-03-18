# HCR Dialogue System

## Installation

```bash
conda create -n hcr python=3.10
conda activate hcr
pip install -r requirements.txt
```

For microphone usage (Ubuntu):

```bash
sudo apt-get install -y libportaudio2 portaudio19-dev
python -c "import sounddevice as sd; print(sd.query_devices())"
```

## Quick Start

### Train NLU

```bash
# BERT (default)
./scripts/train_nlu.sh --backend bert

# FastText
./scripts/train_nlu.sh --backend fasttext
```

### Evaluate NLU

```bash
# BERT (default)
python eval/nlu_eval.py

# FastText
python eval/nlu_eval.py --backend fasttext --model models/nlu_intent_fast_text.bin
```

### Run Demo

```bash
# Simulation (no hardware needed)
./demo.sh

# Microphone
./demo_microphone.sh
```

### Generate Voice Prompts

```bash
python -m src.utils.generate_prompt
```

## Project Structure

```
├── demo.sh / demo_microphone.sh   # Demo entry scripts
├── scripts/train_nlu.sh           # NLU training script
├── eval/nlu_eval.py               # NLU evaluation
├── src/
│   ├── dialogue_manager.py        # Main dialogue pipeline
│   └── utils/
│       ├── nlu_intent_bert.py     # BERT intent classifier
│       ├── nlu_intent_fast_text.py
│       ├── speech_to_text.py      # STT (Vosk)
│       ├── text_to_speech.py      # TTS (Piper)
│       └── generate_prompt.py     # Voice prompt generator
├── models/                        # Trained model artifacts
├── nlu_data/                      # Train / test data
└── voice_data/                    # Generated voice prompts
```
