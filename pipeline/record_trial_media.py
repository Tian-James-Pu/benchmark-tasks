"""Keep each trial's score, source fingerprint, and rendered media together."""

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
from tempfile import NamedTemporaryFile


SOURCE_FORMATS = {".py", ".json", ".yaml", ".yml", ".usd", ".usda", ".usdc", ".obj", ".stl", ".npz"}


def digest(path):
    with path.open("rb") as contents:
        return hashlib.file_digest(contents, "sha256").hexdigest()


def sources(directory):
    return {
        path.relative_to(directory).as_posix(): digest(path)
        for path in sorted(directory.rglob("*"))
        if path.is_file() and path.suffix.lower() in SOURCE_FORMATS and path.name != "task.json"
    }


def copy_view(source, destination):
    with NamedTemporaryFile(dir=destination.parent, prefix=".trial-media-", delete=False) as file:
        staged = Path(file.name)
    try:
        shutil.copy2(source, staged)
        staged.replace(destination)
    finally:
        staged.unlink(missing_ok=True)


def capture(directory, output, task_file, instruction):
    task_file = Path(task_file).as_posix()
    initial = {
        "entry_file": task_file,
        "instruction_sha256": hashlib.sha256(instruction.encode()).hexdigest(),
        "source_files_sha256": sources(directory),
    }
    if task_file not in initial["source_files_sha256"]:
        raise ValueError(f"Cannot fingerprint task entry file {task_file}")
    (output / "trial-input.json").write_text(json.dumps(initial, indent=2) + "\n")


def finish(directory, output, record_path, name, shared):
    initial = json.loads((output / "trial-input.json").read_text())
    native = json.loads(record_path.read_text())
    media = output / "submission-media"
    artifacts = [media / f"{name}.mp4", media / f"{name}-sheet.png"]
    if name == "demo":
        artifacts.append(media / "setup.png")
    final_sources = sources(directory)
    receipt = {
        **initial,
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "trial_record": record_path.relative_to(output).as_posix(),
        "trial_record_sha256": digest(record_path),
        "policy": native["eval"].get("policy"),
        "terminal_status": native["status"],
        "controls": native["stats"].get("total_steps"),
        "task_score": (native.get("results") or {}).get("metrics", {}).get("task_score"),
        "same_source_fingerprints_at_finish": initial["source_files_sha256"] == final_sources,
        "closing_source_files_sha256": final_sources,
        "media": {path.relative_to(output).as_posix(): digest(path) for path in artifacts},
    }
    saved = output / "trial-media.json"
    saved.write_text(json.dumps(receipt, indent=2) + "\n")
    with (shared / ".media.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for path in artifacts:
            copy_view(path, shared / path.name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="stage", required=True)
    start = commands.add_parser("capture")
    start.add_argument("directory", type=Path)
    start.add_argument("output", type=Path)
    start.add_argument("task_file")
    start.add_argument("instruction")
    done = commands.add_parser("finish")
    done.add_argument("directory", type=Path)
    done.add_argument("output", type=Path)
    done.add_argument("record_path", type=Path)
    done.add_argument("name")
    done.add_argument("shared", type=Path)
    args = parser.parse_args()
    stage = vars(args)
    operation = stage.pop("stage")
    if operation == "capture":
        capture(**stage)
    else:
        finish(**stage)
