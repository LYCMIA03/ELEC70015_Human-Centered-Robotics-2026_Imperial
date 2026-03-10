"""DistilBERT-based NLU intent classifier.

Inference path uses ONNX Runtime (no torch required at runtime).
Training path lazily imports torch/transformers training utilities.
"""

import argparse
from collections import Counter
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from transformers import DistilBertTokenizerFast

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "nlu_intent_bert"
DEFAULT_TRAIN_DATA = PROJECT_ROOT / "nlu_data" / "train.txt"
DEFAULT_TEST_DATA = PROJECT_ROOT / "nlu_data" / "test.txt"
BASE_MODEL = "distilbert-base-uncased"
ONNX_FILENAME = "model.onnx"

LABEL2ID = {"affirmative": 0, "negative": 1, "other": 2}
ID2LABEL = {v: k for k, v in LABEL2ID.items()}


class Intent(str, Enum):
    AFFIRMATIVE = "affirmative"
    NEGATIVE = "negative"
    OTHER = "other"


class IntentClassifierBert:
    """DistilBERT intent classifier.

    - load()  → ONNX Runtime inference (preferred, no torch needed)
    - train() → PyTorch fine-tuning (torch lazily imported)
    - save()  → saves HuggingFace weights + exports ONNX automatically
    """

    def __init__(
        self,
        model=None,
        tokenizer=None,
        ort_session=None,
        device: Optional[str] = None,
    ):
        self._model = model
        self._tokenizer = tokenizer
        self._ort_session = ort_session
        self._device = device or "cpu"
        if self._model is not None:
            self._model.to(self._device)

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def load(cls, model_path: str = str(DEFAULT_MODEL_PATH)) -> "IntentClassifierBert":
        """Load model. Uses ONNX Runtime if model.onnx exists, else PyTorch."""
        path = Path(model_path)
        if not path.exists():
            raise FileNotFoundError(f"Model not found: {path}")
        tokenizer = DistilBertTokenizerFast.from_pretrained(str(path))
        onnx_path = path / ONNX_FILENAME
        if onnx_path.exists():
            import onnxruntime as ort
            session = ort.InferenceSession(str(onnx_path))
            print(f"  [NLU] Loaded ONNX model from {onnx_path}")
            return cls(tokenizer=tokenizer, ort_session=session)
        # Fallback: PyTorch weights
        import torch
        from transformers import DistilBertForSequenceClassification
        model = DistilBertForSequenceClassification.from_pretrained(str(path))
        model.eval()
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"  [NLU] Loaded PyTorch model from {path} (device={device})")
        return cls(model=model, tokenizer=tokenizer, device=device)

    @classmethod
    def train(
        cls,
        data_path: Optional[str] = None,
        epoch: int = 5,
        lr: float = 2e-5,
        batch_size: int = 32,
        max_length: int = 64,
        base_model: str = BASE_MODEL,
    ) -> "IntentClassifierBert":
        import torch
        from torch.utils.data import DataLoader, Dataset
        from transformers import (
            DistilBertForSequenceClassification,
            get_linear_schedule_with_warmup,
        )

        class _IntentDataset(Dataset):
            def __init__(self, encodings, labels):
                self.encodings = encodings
                self.labels = labels

            def __len__(self):
                return len(self.labels)

            def __getitem__(self, idx):
                item = {k: torch.tensor(v[idx]) for k, v in self.encodings.items()}
                item["labels"] = torch.tensor(self.labels[idx])
                return item

        train_file = data_path or str(DEFAULT_TRAIN_DATA)
        if not Path(train_file).exists():
            raise FileNotFoundError(f"Training file not found: {train_file}")

        cls._report_data_diagnostics(train_file=train_file, eval_file=str(DEFAULT_TEST_DATA))

        samples = cls._load_labeled_samples(train_file)
        texts = [text for _, text in samples]
        labels = [LABEL2ID.get(label, LABEL2ID["other"]) for label, _ in samples]

        tokenizer = DistilBertTokenizerFast.from_pretrained(base_model)
        encodings = tokenizer(texts, truncation=True, padding=True, max_length=max_length)

        dataset = _IntentDataset(encodings, labels)
        loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

        model = DistilBertForSequenceClassification.from_pretrained(
            base_model,
            num_labels=len(LABEL2ID),
            id2label=ID2LABEL,
            label2id=LABEL2ID,
        )
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model.to(device)

        optimizer = torch.optim.AdamW(model.parameters(), lr=lr)
        total_steps = len(loader) * epoch
        scheduler = get_linear_schedule_with_warmup(
            optimizer,
            num_warmup_steps=total_steps // 10,
            num_training_steps=total_steps,
        )

        for ep in range(1, epoch + 1):
            model.train()
            total_loss = 0.0
            for batch in loader:
                optimizer.zero_grad()
                outputs = model(
                    input_ids=batch["input_ids"].to(device),
                    attention_mask=batch["attention_mask"].to(device),
                    labels=batch["labels"].to(device),
                )
                outputs.loss.backward()
                optimizer.step()
                scheduler.step()
                total_loss += outputs.loss.item()
            print(f"  Epoch {ep}/{epoch} - loss: {total_loss / len(loader):.4f}")

        model.eval()
        return cls(model=model, tokenizer=tokenizer, device=device)

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def predict(self, text: str) -> Tuple[Intent, float]:
        if self._tokenizer is None or (self._model is None and self._ort_session is None):
            raise RuntimeError("Model not loaded. Call train() or load() first.")
        scores = self._get_scores(text)
        idx = int(scores.argmax())
        return self._to_intent(ID2LABEL[idx]), float(scores[idx])

    def predict_all(self, text: str) -> List[Tuple[Intent, float]]:
        if self._tokenizer is None or (self._model is None and self._ort_session is None):
            raise RuntimeError("Model not loaded. Call train() or load() first.")
        scores = self._get_scores(text)
        return [
            (self._to_intent(ID2LABEL[int(i)]), float(scores[i]))
            for i in scores.argsort()[::-1]
        ]

    def _get_scores(self, text: str) -> np.ndarray:
        """Run forward pass and return softmax probabilities as numpy array."""
        enc = self._tokenizer(
            text.strip(), return_tensors="np", truncation=True, padding=True, max_length=64
        )
        if self._ort_session is not None:
            logits = self._ort_session.run(
                ["logits"],
                {"input_ids": enc["input_ids"], "attention_mask": enc["attention_mask"]},
            )[0][0]
        else:
            import torch
            pt_enc = {k: torch.tensor(v).to(self._device) for k, v in enc.items()}
            with torch.no_grad():
                logits = self._model(**pt_enc).logits[0].cpu().numpy()
        e = np.exp(logits - logits.max())
        return e / e.sum()

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save(self, model_path: str = str(DEFAULT_MODEL_PATH)) -> Path:
        """Save HuggingFace weights and export ONNX in one step."""
        if self._model is None or self._tokenizer is None:
            raise RuntimeError("No PyTorch model to save.")
        path = Path(model_path)
        path.mkdir(parents=True, exist_ok=True)
        self._model.save_pretrained(str(path))
        self._tokenizer.save_pretrained(str(path))
        self._export_onnx(path)
        return path

    def _export_onnx(self, model_dir: Path) -> Path:
        import torch
        onnx_path = model_dir / ONNX_FILENAME
        dummy = self._tokenizer(
            "export", return_tensors="pt",
            padding="max_length", max_length=64, truncation=True,
        )
        with torch.no_grad():
            torch.onnx.export(
                self._model,
                (
                    dummy["input_ids"].to(self._device),
                    dummy["attention_mask"].to(self._device),
                ),
                str(onnx_path),
                input_names=["input_ids", "attention_mask"],
                output_names=["logits"],
                dynamic_axes={
                    "input_ids":      {0: "batch", 1: "sequence"},
                    "attention_mask": {0: "batch", 1: "sequence"},
                },
                opset_version=14,
                dynamo=False,
            )
        print(f"  ONNX model exported to: {onnx_path}")
        return onnx_path

    @classmethod
    def export_onnx(cls, model_path: str = str(DEFAULT_MODEL_PATH)) -> Path:
        """Convert an existing HuggingFace model directory to ONNX (one-off utility)."""
        import torch
        from transformers import DistilBertForSequenceClassification
        path = Path(model_path)
        if not path.exists():
            raise FileNotFoundError(f"Model not found: {path}")
        tokenizer = DistilBertTokenizerFast.from_pretrained(str(path))
        model = DistilBertForSequenceClassification.from_pretrained(str(path))
        model.eval()
        device = "cuda" if torch.cuda.is_available() else "cpu"
        obj = cls(model=model, tokenizer=tokenizer, device=device)
        return obj._export_onnx(path)

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------

    def evaluate(self, test_path: str = str(DEFAULT_TEST_DATA)):
        if self._tokenizer is None or (self._model is None and self._ort_session is None):
            raise RuntimeError("Model not loaded.")
        test_file = Path(test_path)
        if not test_file.exists():
            raise FileNotFoundError(f"Test file not found: {test_file}")

        samples = self._load_labeled_samples(str(test_file))
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

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _to_intent(label: str) -> Intent:
        try:
            return Intent(label)
        except ValueError:
            return Intent.OTHER

    @staticmethod
    def _load_labeled_samples(file_path: str) -> List[Tuple[str, str]]:
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


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NLU Intent Classifier (DistilBERT)")
    sub = parser.add_subparsers(dest="command", required=True)

    p_train = sub.add_parser("train", help="Fine-tune a new model")
    p_train.add_argument("--data", type=str, default=None)
    p_train.add_argument("--output", type=str, default=str(DEFAULT_MODEL_PATH))
    p_train.add_argument("--epoch", type=int, default=5)
    p_train.add_argument("--lr", type=float, default=2e-5)
    p_train.add_argument("--batch-size", type=int, default=32)
    p_train.add_argument("--max-length", type=int, default=64)
    p_train.add_argument("--base-model", type=str, default=BASE_MODEL)

    p_export = sub.add_parser("export-onnx", help="Export existing HuggingFace model to ONNX")
    p_export.add_argument("--model", type=str, default=str(DEFAULT_MODEL_PATH))

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
        print("Fine-tuning DistilBERT intent classifier...")
        classifier = IntentClassifierBert.train(
            data_path=args.data,
            epoch=args.epoch,
            lr=args.lr,
            batch_size=args.batch_size,
            max_length=args.max_length,
            base_model=args.base_model,
        )
        path = classifier.save(args.output)
        print(f"Model saved to: {path}")
    elif args.command == "export-onnx":
        path = IntentClassifierBert.export_onnx(args.model)
        print(f"ONNX exported to: {path}")
    elif args.command == "predict":
        classifier = IntentClassifierBert.load(args.model)
        intent, confidence = classifier.predict(args.text)
        print(f"Text:       {args.text}")
        print(f"Intent:     {intent.value}")
        print(f"Confidence: {confidence:.4f}")
        print("\nAll intents:")
        for i, score in classifier.predict_all(args.text):
            print(f"  {i.value:15s} {score:.4f}")
    elif args.command == "evaluate":
        classifier = IntentClassifierBert.load(args.model)
        classifier.evaluate(args.test)
    elif args.command == "interactive":
        classifier = IntentClassifierBert.load(args.model)
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
