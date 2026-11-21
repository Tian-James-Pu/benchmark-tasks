"""Copy and verify an isolated task package before the submission script uses it."""

import hashlib
import json
from pathlib import Path
import re
import shutil
import sys


MEDIA_NAMES = {"demo.mp4", "dream.mp4", "setup.png"}


def inventory(directory, *, skip_bytecode=False):
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError(f"expected a regular directory: {directory}")
    result = set()
    for path in directory.rglob("*"):
        relative = path.relative_to(directory)
        if path.is_symlink():
            raise ValueError(f"unexpected symlink: {relative}")
        if skip_bytecode and "__pycache__" in relative.parts:
            continue
        if path.is_file():
            result.add(relative.as_posix())
        elif not path.is_dir():
            raise ValueError(f"unexpected special file: {relative}")
    return result


def copy_verified(source, destination, expected_hash):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    with destination.open("rb") as handle:
        actual_hash = hashlib.file_digest(handle, "sha256").hexdigest()
    if actual_hash != expected_hash:
        raise ValueError(f"SHA-256 mismatch for {source.name}")


def stage(package, task, destination):
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_-]*", task):
        raise ValueError("invalid task folder name")
    manifest_path = package / "manifest.json"
    tasks = package / "tasks"
    if manifest_path.is_symlink() or tasks.is_symlink():
        raise ValueError("manifest.json and tasks/ must be local package entries")
    manifest = json.loads(manifest_path.read_text())
    source = tasks / task
    source_files = inventory(source, skip_bytecode=True)
    source_hashes = manifest["source_sha256"]
    if not {"README.md", "task.json", "task.py"} <= source_files or source_files != set(source_hashes):
        raise ValueError("task file inventory differs from manifest")
    media = package / "media"
    media_hashes = manifest["media"]
    if inventory(media) != MEDIA_NAMES or set(media_hashes) != MEDIA_NAMES:
        raise ValueError("media file inventory differs from manifest")

    for relative in sorted(source_files):
        copy_verified(source / relative, destination / "tasks" / task / relative, source_hashes[relative])
    for name in sorted(MEDIA_NAMES):
        copy_verified(media / name, destination / "media" / name, media_hashes[name]["sha256"])


if __name__ == "__main__":
    try:
        stage(Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit(f"submit: invalid staged package: {exc}")
