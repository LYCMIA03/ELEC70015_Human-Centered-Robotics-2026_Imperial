"""NLU evaluation script — supports both DistilBERT and FastText backends.

Usage:
    # BERT (default)
    python eval/nlu_eval.py
    python eval/nlu_eval.py --backend bert  --model models/nlu_intent_bert

    # FastText
    python eval/nlu_eval.py --backend fasttext --model models/nlu_intent_fast_text.bin
"""

import argparse
import sys
from pathlib import Path

# Allow running from project root without installing the package
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.utils.nlu_intent_bert import Intent

BERT_DEFAULT_MODEL     = str(PROJECT_ROOT / "models" / "nlu_intent_bert")
FASTTEXT_DEFAULT_MODEL = str(PROJECT_ROOT / "models" / "nlu_intent.bin")
DEFAULT_TEST           = str(PROJECT_ROOT / "nlu_data" / "test.txt")

CLASSES = [i.value for i in Intent]


# ---------------------------------------------------------------------------
# Backend loader
# ---------------------------------------------------------------------------

def load_classifier(backend: str, model_path: str):
    """Return a loaded classifier for the given backend."""
    if backend == "bert":
        from src.utils.nlu_intent_bert import IntentClassifierBert
        return IntentClassifierBert.load(model_path)
    if backend == "fasttext":
        from src.utils.nlu_intent_fast_text import IntentClassifier
        return IntentClassifier.load(model_path)
    raise ValueError(f"Unknown backend: {backend!r}. Choose 'bert' or 'fasttext'.")


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_test_data(path: str):
    samples = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and " " in line:
            label, text = line.split(" ", 1)
            samples.append((label.replace("__label__", ""), text))
    return samples


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_metrics(samples, classifier):
    tp = {c: 0 for c in CLASSES}
    fp = {c: 0 for c in CLASSES}
    fn = {c: 0 for c in CLASSES}
    correct = 0
    results = []

    for true_label, text in samples:
        pred_intent, confidence = classifier.predict(text)
        pred_label = pred_intent.value
        hit = pred_label == true_label
        if hit:
            correct += 1
            tp[true_label] += 1
        else:
            fp[pred_label] += 1
            fn[true_label] += 1
        results.append((text, true_label, pred_label, confidence, hit))

    per_class = {}
    for c in CLASSES:
        p = tp[c] / (tp[c] + fp[c]) if (tp[c] + fp[c]) > 0 else 0.0
        r = tp[c] / (tp[c] + fn[c]) if (tp[c] + fn[c]) > 0 else 0.0
        f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
        per_class[c] = dict(precision=p, recall=r, f1=f1, tp=tp[c], fp=fp[c], fn=fn[c])

    accuracy = correct / len(samples) if samples else 0.0
    return accuracy, per_class, results


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def print_summary(accuracy, per_class, results, verbose=False):
    total = len(results)
    correct = sum(1 for *_, hit in results if hit)

    print(f"\n{'=' * 64}")
    print(f"  Accuracy : {correct}/{total}  ({accuracy:.2%})")
    print(f"{'=' * 64}")

    # Per-class table
    print(f"\n  {'Class':15s} {'P':>8s} {'R':>8s} {'F1':>8s}  {'TP':>4s} {'FP':>4s} {'FN':>4s}")
    print(f"  {'-' * 55}")
    for cls, m in per_class.items():
        print(
            f"  {cls:15s} {m['precision']:>8.2%} {m['recall']:>8.2%} {m['f1']:>8.2%}"
            f"  {m['tp']:>4d} {m['fp']:>4d} {m['fn']:>4d}"
        )
    macro_p  = sum(m["precision"] for m in per_class.values()) / len(per_class)
    macro_r  = sum(m["recall"]    for m in per_class.values()) / len(per_class)
    macro_f1 = sum(m["f1"]        for m in per_class.values()) / len(per_class)
    print(f"  {'-' * 55}")
    print(f"  {'macro avg':15s} {macro_p:>8.2%} {macro_r:>8.2%} {macro_f1:>8.2%}")

    # Confusion matrix
    print(f"\n  Confusion matrix (row=true, col=pred):")
    print(f"  {'':15s}", end="")
    for c in CLASSES:
        print(f"  {c[:6]:>6s}", end="")
    print()
    cm = {t: {p: 0 for p in CLASSES} for t in CLASSES}
    for _, true_label, pred_label, _, _ in results:
        cm[true_label][pred_label] += 1
    for t in CLASSES:
        print(f"  {t:15s}", end="")
        for p in CLASSES:
            val = cm[t][p]
            mark = f"[{val:3d}]" if t == p else f" {val:3d} "
            print(f"  {mark}", end="")
        print()

    # Confidence distribution per class
    import statistics
    print(f"\n  Confidence (correct predictions):")
    print(f"  {'Class':15s} {'Mean':>8s} {'Median':>8s} {'Min':>8s}")
    print(f"  {'-' * 45}")
    for cls in CLASSES:
        confs = [conf for _, true, pred, conf, hit in results if hit and true == cls]
        if confs:
            print(
                f"  {cls:15s} {statistics.mean(confs):>8.4f}"
                f" {statistics.median(confs):>8.4f}"
                f" {min(confs):>8.4f}"
            )

    # Misclassifications
    errors = [(text, true, pred, conf) for text, true, pred, conf, hit in results if not hit]
    print(f"\n  Misclassified: {len(errors)}/{total}")
    if errors:
        print(f"  {'-' * 60}")
        for text, true, pred, conf in errors:
            label = f"true={true}, pred={pred}, conf={conf:.4f}"
            # Truncate long texts for display
            display = (text[:72] + "...") if len(text) > 75 else text
            print(f"  [{label}]")
            print(f"    \"{display}\"")

    if verbose and not errors:
        print("  (none)")

    print()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate NLU intent classifier")
    parser.add_argument(
        "--backend", default="bert", choices=["bert", "fasttext"],
        help="Model backend to evaluate (default: bert)",
    )
    parser.add_argument("--model",   default=None,         help="Path to model (auto-selected by backend if omitted)")
    parser.add_argument("--test",    default=DEFAULT_TEST,  help="Path to test data file")
    parser.add_argument("--verbose", action="store_true",   help="Show all predictions")
    return parser.parse_args()


def main():
    args = parse_args()

    # Resolve default model path per backend
    if args.model is None:
        args.model = BERT_DEFAULT_MODEL if args.backend == "bert" else FASTTEXT_DEFAULT_MODEL

    print(f"[Eval] Backend : {args.backend}")
    print(f"[Eval] Model   : {args.model}")
    print(f"[Eval] Test    : {args.test}")

    print("[Eval] Loading model...")
    classifier = load_classifier(args.backend, args.model)

    print("[Eval] Loading test data...")
    samples = load_test_data(args.test)
    class_counts = {c: sum(1 for lbl, _ in samples if lbl == c) for c in CLASSES}
    print(f"[Eval] Samples: {len(samples)}  " +
          "  ".join(f"{c}={n}" for c, n in class_counts.items()))

    print("[Eval] Running inference...")
    accuracy, per_class, results = compute_metrics(samples, classifier)

    print_summary(accuracy, per_class, results, verbose=args.verbose)

    if args.verbose:
        print("  All predictions:")
        print(f"  {'True':15s} {'Pred':15s} {'Conf':>6s}  Text")
        print(f"  {'-' * 70}")
        for text, true, pred, conf, hit in results:
            flag = "  " if hit else "✗ "
            display = (text[:50] + "...") if len(text) > 53 else text
            print(f"  {flag}{true:15s} {pred:15s} {conf:>6.4f}  {display}")


if __name__ == "__main__":
    main()
