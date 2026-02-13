#!/usr/bin/env bash
python -m src.dialogue_manager \
  --first-user-wav "voice_data/sim_user_answer_other_a.wav" \
  --second-user-wav "voice_data/sim_user_answer_affirmative_b.wav" \
  --nlu-model "models/nlu_intent.bin" \
  --no-play
