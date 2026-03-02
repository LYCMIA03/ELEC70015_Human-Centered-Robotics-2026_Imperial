#!/usr/bin/env bash
# Fine-tune DistilBERT intent classifier and evaluate on the test set.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ---------- hardcoded parameters ----------
EPOCH=10
LR=2e-5
BATCH_SIZE=1
MAX_LENGTH=64
BASE_MODEL="distilbert-base-uncased"
DATA="${PROJECT_ROOT}/nlu_data/train.txt"
OUTPUT="${PROJECT_ROOT}/models/nlu_intent_bert"
SKIP_EVAL=0

NLU_SCRIPT="${PROJECT_ROOT}/src/utils/nlu_intent_bert.py"
TEST_DATA="${PROJECT_ROOT}/nlu_data/test.txt"

echo "============================================"
echo "  NLU Training (DistilBERT)"
echo "============================================"
echo "  base model : ${BASE_MODEL}"
echo "  train data : ${DATA}"
echo "  output dir : ${OUTPUT}"
echo "  epochs     : ${EPOCH}"
echo "  lr         : ${LR}"
echo "  batch size : ${BATCH_SIZE}"
echo "  max length : ${MAX_LENGTH}"
echo "============================================"

python3 "${NLU_SCRIPT}" train \
    --data       "${DATA}"       \
    --output     "${OUTPUT}"     \
    --epoch      "${EPOCH}"      \
    --lr         "${LR}"         \
    --batch-size "${BATCH_SIZE}" \
    --max-length "${MAX_LENGTH}" \
    --base-model "${BASE_MODEL}"

if [[ "${SKIP_EVAL}" -eq 0 ]]; then
    echo ""
    echo "============================================"
    echo "  Evaluating on test set"
    echo "============================================"
    python3 "${NLU_SCRIPT}" evaluate \
        --model "${OUTPUT}" \
        --test  "${TEST_DATA}"
fi
