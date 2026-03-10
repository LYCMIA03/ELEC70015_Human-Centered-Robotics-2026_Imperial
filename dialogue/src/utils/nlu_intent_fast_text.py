"""FastText-based NLU intent classifier."""

import argparse
from collections import Counter
from enum import Enum
from pathlib import Path
from typing import Optional

import numpy as np

_original_np_array = np.array


def _patched_np_array(obj, *args, copy=None, **kwargs):
    if copy is False:
        return np.asarray(obj, *args, **kwargs)
    return _original_np_array(obj, *args, copy=copy, **kwargs)


np.array = _patched_np_array

import fasttext

fasttext.FastText.eprint = lambda x: None

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "nlu_intent.bin"
DEFAULT_TRAIN_DATA = PROJECT_ROOT / "nlu_data" / "train.txt"
DEFAULT_TEST_DATA = PROJECT_ROOT / "nlu_data" / "test.txt"


class Intent(str, Enum):
    AFFIRMATIVE = "affirmative"
    NEGATIVE = "negative"
    OTHER = "other"


class IntentClassifier:
    """FastText-based three-class intent classifier."""

    def __init__(self, model: Optional[fasttext.FastText._FastText] = None):
        self._model = model

    @classmethod
    def load(cls, model_path: str = str(DEFAULT_MODEL_PATH)) -> "IntentClassifier":
        path = Path(model_path)
        if not path.exists():
            raise FileNotFoundError(f"Model not found: {path}")
        return cls(model=fasttext.load_model(str(path)))

    @classmethod
    def train(
        cls,
        data_path: Optional[str] = None,
        epoch: int = 300,
        lr: float = 0.1,
        word_ngrams: int = 2,
        dim: int = 200,
        bucket: int = 20000,
        minn: int = 2,
        maxn: int = 4,
        thread: int = 1,
    ) -> "IntentClassifier":
        if data_path is not None:
            train_file = str(data_path)
        elif DEFAULT_TRAIN_DATA.exists():
            train_file = str(DEFAULT_TRAIN_DATA)
        else:
            raise FileNotFoundError(f"Training file not found: {DEFAULT_TRAIN_DATA}")

        cls._report_data_diagnostics(train_file=train_file, eval_file=str(DEFAULT_TEST_DATA))

        model = fasttext.train_supervised(
            input=train_file,
            epoch=epoch,
            lr=lr,
            wordNgrams=word_ngrams,
            dim=dim,
            loss="softmax",
            bucket=bucket,
            minn=minn,
            maxn=maxn,
            thread=thread,
        )
        return cls(model=model)

    def predict(self, text: str):
        if self._model is None:
            raise RuntimeError("Model not loaded. Call train() or load() first.")
        text = text.strip().lower()
        labels, scores = self._model.predict(text, k=1)
        label_str = labels[0].replace("__label__", "")
        try:
            intent = Intent(label_str)
        except ValueError:
            intent = Intent.OTHER
        return intent, float(scores[0])

    def predict_all(self, text: str):
        if self._model is None:
            raise RuntimeError("Model not loaded. Call train() or load() first.")
        text = text.strip().lower()
        labels, scores = self._model.predict(text, k=3)
        results = []
        for label, score in zip(labels, scores):
            label_str = label.replace("__label__", "")
            try:
                intent = Intent(label_str)
            except ValueError:
                intent = Intent.OTHER
            results.append((intent, float(score)))
        return results

    def save(self, model_path: str = str(DEFAULT_MODEL_PATH)) -> Path:
        if self._model is None:
            raise RuntimeError("No model to save.")
        path = Path(model_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._model.save_model(str(path))
        return path

    def evaluate(self, test_path: str):
        if self._model is None:
            raise RuntimeError("Model not loaded.")
        test_file = Path(test_path)
        if not test_file.exists():
            raise FileNotFoundError(f"Test file not found: {test_file}")

        samples = []
        with open(test_file, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                label, text = line.split(" ", 1)
                samples.append((label.replace("__label__", ""), text))

        all_labels = [i.value for i in Intent]
        tp = {l: 0 for l in all_labels}
        fp = {l: 0 for l in all_labels}
        fn = {l: 0 for l in all_labels}
        correct = 0
        errors = []

        for true_label, text in samples:
            pred_intent, confidence = self.predict(text)
            pred_label = pred_intent.value
            if pred_label == true_label:
                correct += 1
                tp[true_label] += 1
            else:
                fp[pred_label] += 1
                fn[true_label] += 1
                errors.append((text, true_label, pred_label, confidence))

        total = len(samples)
        accuracy = correct / total if total > 0 else 0.0
        per_class = {}
        for label in all_labels:
            p = tp[label] / (tp[label] + fp[label]) if (tp[label] + fp[label]) > 0 else 0.0
            r = tp[label] / (tp[label] + fn[label]) if (tp[label] + fn[label]) > 0 else 0.0
            f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
            per_class[label] = {"precision": p, "recall": r, "f1": f1}

        print(f"\n{'=' * 60}")
        print(f"  Test Results: {correct}/{total} correct")
        print(f"  Accuracy: {accuracy:.2%}")
        print(f"{'=' * 60}")
        print(f"\n  {'Class':15s} {'Precision':>10s} {'Recall':>10s} {'F1':>10s}")
        print(f"  {'-' * 45}")
        for label, metrics in per_class.items():
            print(
                f"  {label:15s} {metrics['precision']:>10.2%}"
                f" {metrics['recall']:>10.2%} {metrics['f1']:>10.2%}"
            )

        if errors:
            print(f"\n  Misclassified ({len(errors)}):")
            print(f"  {'-' * 55}")
            for text, true, pred, conf in errors:
                print(f'  "{text}"')
                print(f"    true={true}, pred={pred} (conf={conf:.4f})")

        return {"accuracy": accuracy, "per_class": per_class, "errors": errors}

    @staticmethod
    def _load_labeled_samples(file_path: str):
        samples = []
        path = Path(file_path)
        if not path.exists():
            return samples
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or " " not in line:
                    continue
                label, text = line.split(" ", 1)
                samples.append((label.replace("__label__", ""), text))
        return samples

    @classmethod
    def _report_data_diagnostics(cls, train_file: str, eval_file: Optional[str] = None):
        train_samples = cls._load_labeled_samples(train_file)
        train_counter = Counter(label for label, _ in train_samples)
        print("Dataset diagnostics:")
        print(
            "  Train counts -> "
            f"affirmative={train_counter.get('affirmative', 0)}, "
            f"negative={train_counter.get('negative', 0)}, "
            f"other={train_counter.get('other', 0)}"
        )
        if eval_file:
            eval_samples = cls._load_labeled_samples(eval_file)
            eval_counter = Counter(label for label, _ in eval_samples)
            print(
                "  Eval counts  -> "
                f"affirmative={eval_counter.get('affirmative', 0)}, "
                f"negative={eval_counter.get('negative', 0)}, "
                f"other={eval_counter.get('other', 0)}"
            )
            train_texts = {text for _, text in train_samples}
            eval_texts = {text for _, text in eval_samples}
            overlap = len(train_texts.intersection(eval_texts))
            print(f"  Exact text overlap(train/eval): {overlap}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NLU Intent Classifier")
    sub = parser.add_subparsers(dest="command", required=True)

    p_train = sub.add_parser("train", help="Train a new model")
    p_train.add_argument("--data", type=str, default=None, help="Training data file")
    p_train.add_argument("--output", type=str, default=str(DEFAULT_MODEL_PATH))
    p_train.add_argument("--epoch", type=int, default=300)
    p_train.add_argument("--lr", type=float, default=0.1)
    p_train.add_argument("--word-ngrams", type=int, default=2)
    p_train.add_argument("--dim", type=int, default=200)
    p_train.add_argument("--bucket", type=int, default=20000)
    p_train.add_argument("--minn", type=int, default=2)
    p_train.add_argument("--maxn", type=int, default=4)
    p_train.add_argument("--thread", type=int, default=1)

    p_pred = sub.add_parser("predict", help="Predict intent for text")
    p_pred.add_argument("--text", type=str, required=True)
    p_pred.add_argument("--model", type=str, default=str(DEFAULT_MODEL_PATH))

    p_eval = sub.add_parser("evaluate", help="Evaluate model on a test set")
    p_eval.add_argument("--test", type=str, default=str(DEFAULT_TEST_DATA))
    p_eval.add_argument("--model", type=str, default=str(DEFAULT_MODEL_PATH))

    p_inter = sub.add_parser("interactive", help="Interactive prediction mode")
    p_inter.add_argument("--model", type=str, default=str(DEFAULT_MODEL_PATH))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.command == "train":
        print("Training intent classifier...")
        classifier = IntentClassifier.train(
            data_path=args.data,
            epoch=args.epoch,
            lr=args.lr,
            word_ngrams=args.word_ngrams,
            dim=args.dim,
            bucket=args.bucket,
            minn=args.minn,
            maxn=args.maxn,
            thread=args.thread,
        )
        path = classifier.save(args.output)
        print(f"Model saved to: {path}")
    elif args.command == "predict":
        classifier = IntentClassifier.load(args.model)
        intent, confidence = classifier.predict(args.text)
        print(f"Text:       {args.text}")
        print(f"Intent:     {intent.value}")
        print(f"Confidence: {confidence:.4f}")
        print("\nAll intents:")
        for i, score in classifier.predict_all(args.text):
            print(f"  {i.value:15s} {score:.4f}")
    elif args.command == "evaluate":
        classifier = IntentClassifier.load(args.model)
        classifier.evaluate(args.test)
    elif args.command == "interactive":
        classifier = IntentClassifier.load(args.model)
        print("NLU Interactive Mode (type 'quit' to exit)")
        print("-" * 40)
        while True:
            try:
                text = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nBye.")
                break
            if text.lower() in ("quit", "exit", "q"):
                break
            if not text:
                continue
            intent, confidence = classifier.predict(text)
            print(f"  -> {intent.value} ({confidence:.4f})")


if __name__ == "__main__":
    main()
