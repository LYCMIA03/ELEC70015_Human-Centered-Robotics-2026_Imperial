"""WAV file sampling and playback utilities."""

import glob
import random
import shutil
import subprocess
from functools import lru_cache
from pathlib import Path
from typing import List, Optional, Tuple, Union


@lru_cache(maxsize=16)
def _resolve_wav_candidates(wav_spec: str) -> Tuple[Path, ...]:
    if "*" not in wav_spec and "?" not in wav_spec and "[" not in wav_spec:
        return (Path(wav_spec),)
    return tuple(Path(p) for p in sorted(glob.glob(wav_spec)) if Path(p).is_file())


def sample_wav(wav_spec: Union[str, Path]) -> Path:
    """Pick a random WAV file matching *wav_spec* (may contain glob patterns)."""
    candidates = _resolve_wav_candidates(str(wav_spec))
    if not candidates:
        raise FileNotFoundError(f"No WAV file matched: {wav_spec}")
    selected_wav = random.choice(candidates)
    print(f"[Sample Voice] spec='{wav_spec}' -> using '{selected_wav}'")
    return selected_wav


_wav_player_cmd: Optional[List[str]] = None


def _detect_wav_player() -> List[str]:
    """Probe once for an available WAV player; result is reused for all calls."""
    for name, extra_args in [("aplay", []), ("ffplay", ["-autoexit", "-nodisp"])]:
        if shutil.which(name):
            return [name] + extra_args
    raise RuntimeError("No available WAV player found. Install `aplay` or `ffplay`.")


def play_wav(wav_path: Union[str, Path]) -> None:
    """Play a single resolved WAV file."""
    global _wav_player_cmd
    if _wav_player_cmd is None:
        _wav_player_cmd = _detect_wav_player()
    subprocess.run(
        _wav_player_cmd + [str(wav_path)],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def speak_robot(wav_spec: Union[str, Path], play_audio: bool) -> None:
    """Sample a WAV from *wav_spec* and optionally play it."""
    selected_wav = sample_wav(wav_spec)
    if play_audio:
        play_wav(selected_wav)
