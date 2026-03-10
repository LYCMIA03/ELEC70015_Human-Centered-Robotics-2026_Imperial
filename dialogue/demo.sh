#!/usr/bin/env bash
python -m src.dialogue_manager \
  --sim \
  --first-user-wav "voice_data/sim_user_answer_other_b.wav" \
  --second-user-wav "voice_data/sim_user_answer_negative_a.wav" \
  --nlu-backend auto \
  --nlu-model "models/nlu_intent_bert" \
  --no-play
