#!/usr/bin/env bash

# Usage: 
# List available input devices:
#   python -m src.utils.speech_to_text --list-devices
# Then pass the device index via --device <N>, e.g.:
#   python -m src.dialogue_manager --nlu-backend auto --nlu-model "models/nlu_intent_bert" --device 6

python -m src.dialogue_manager \
  --nlu-backend auto \
  --nlu-model "models/nlu_intent_bert" \
  --device 24
