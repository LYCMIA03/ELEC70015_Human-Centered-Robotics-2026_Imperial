#!/usr/bin/env bash
# Train NLU intent classifier (DistilBERT or FastText) and evaluate on the test set.
#
# Usage:
#   ./scripts/train_nlu.sh [options]
#
# Options:
#   --backend    <bert|fasttext>   Which backend to train      (default: bert)
#
# DistilBERT options:
#   --epoch      <int>             Training epochs             (default: 10)
#   --lr         <float>           Learning rate               (default: 2e-5)
#   --batch-size <int>             Batch size                  (default: 1)
#   --max-length <int>             Max token length            (default: 64)
#   --base-model <str>             HuggingFace base model      (default: distilbert-base-uncased)
#
# FastText options:
#   --epoch      <int>             Training epochs             (default: 300)
#   --lr         <float>           Learning rate               (default: 0.1)
#   --word-ngrams <int>            Word n-gram size            (default: 2)
#   --dim        <int>             Embedding dimension         (default: 200)
#   --bucket     <int>             Hash bucket size            (default: 20000)
#   --minn       <int>             Min char n-gram             (default: 2)
#   --maxn       <int>             Max char n-gram             (default: 4)
#   --thread     <int>             Training threads            (default: 1)
#
# Common options:
#   --data       <path>            Training data file
#   --output     <path>            Where to save the model
#   --skip-eval                    Skip evaluation after training

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ---------- defaults ----------
BACKEND="bert"

# BERT defaults
EPOCH_BERT=10
LR_BERT=2e-5
BATCH_SIZE=1
MAX_LENGTH=64
BASE_MODEL="distilbert-base-uncased"

# FastText defaults
EPOCH_FT=300
LR_FT=0.1
WORD_NGRAMS=2
DIM=200
BUCKET=20000
MINN=2
MAXN=4
THREAD=1

DATA="${PROJECT_ROOT}/nlu_data/train.txt"
OUTPUT_BERT="${PROJECT_ROOT}/models/nlu_intent_bert"
OUTPUT_FT="${PROJECT_ROOT}/models/nlu_intent_fast_text.bin"
SKIP_EVAL=0

# ---------- parse args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --backend)     BACKEND="$2";      shift 2 ;;
        --epoch)
            EPOCH_BERT="$2"; EPOCH_FT="$2"; shift 2 ;;
        --lr)
            LR_BERT="$2";    LR_FT="$2";    shift 2 ;;
        --batch-size)  BATCH_SIZE="$2";   shift 2 ;;
        --max-length)  MAX_LENGTH="$2";   shift 2 ;;
        --base-model)  BASE_MODEL="$2";   shift 2 ;;
        --word-ngrams) WORD_NGRAMS="$2";  shift 2 ;;
        --dim)         DIM="$2";          shift 2 ;;
        --bucket)      BUCKET="$2";       shift 2 ;;
        --minn)        MINN="$2";         shift 2 ;;
        --maxn)        MAXN="$2";         shift 2 ;;
        --thread)      THREAD="$2";       shift 2 ;;
        --data)        DATA="$2";         shift 2 ;;
        --output)
            OUTPUT_BERT="$2"; OUTPUT_FT="$2"; shift 2 ;;
        --skip-eval)   SKIP_EVAL=1;       shift   ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

TEST_DATA="${PROJECT_ROOT}/nlu_data/test.txt"

# ---------- dispatch ----------
if [[ "${BACKEND}" == "bert" ]]; then
    NLU_SCRIPT="${PROJECT_ROOT}/src/utils/nlu_intent_bert.py"
    OUTPUT="${OUTPUT_BERT}"

    echo "============================================"
    echo "  NLU Training (DistilBERT)"
    echo "============================================"
    echo "  base model : ${BASE_MODEL}"
    echo "  train data : ${DATA}"
    echo "  output dir : ${OUTPUT}"
    echo "  epochs     : ${EPOCH_BERT}"
    echo "  lr         : ${LR_BERT}"
    echo "  batch size : ${BATCH_SIZE}"
    echo "  max length : ${MAX_LENGTH}"
    echo "============================================"

    python3 "${NLU_SCRIPT}" train \
        --data        "${DATA}"       \
        --output      "${OUTPUT}"     \
        --epoch       "${EPOCH_BERT}" \
        --lr          "${LR_BERT}"    \
        --batch-size  "${BATCH_SIZE}" \
        --max-length  "${MAX_LENGTH}" \
        --base-model  "${BASE_MODEL}"

elif [[ "${BACKEND}" == "fasttext" ]]; then
    NLU_SCRIPT="${PROJECT_ROOT}/src/utils/nlu_intent_fast_text.py"
    OUTPUT="${OUTPUT_FT}"

    echo "============================================"
    echo "  NLU Training (FastText)"
    echo "============================================"
    echo "  train data  : ${DATA}"
    echo "  output file : ${OUTPUT}"
    echo "  epochs      : ${EPOCH_FT}"
    echo "  lr          : ${LR_FT}"
    echo "  word-ngrams : ${WORD_NGRAMS}"
    echo "  dim         : ${DIM}"
    echo "============================================"

    python3 "${NLU_SCRIPT}" train \
        --data        "${DATA}"        \
        --output      "${OUTPUT}"      \
        --epoch       "${EPOCH_FT}"    \
        --lr          "${LR_FT}"       \
        --word-ngrams "${WORD_NGRAMS}" \
        --dim         "${DIM}"         \
        --bucket      "${BUCKET}"      \
        --minn        "${MINN}"        \
        --maxn        "${MAXN}"        \
        --thread      "${THREAD}"

else
    echo "Unknown backend: ${BACKEND}. Use 'bert' or 'fasttext'." >&2
    exit 1
fi

if [[ "${SKIP_EVAL}" -eq 0 ]]; then
    echo ""
    echo "============================================"
    echo "  Evaluating on test set"
    echo "============================================"
    python3 "${NLU_SCRIPT}" evaluate \
        --model "${OUTPUT}" \
        --test  "${TEST_DATA}"
fi
