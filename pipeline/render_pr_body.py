"""Render a task PR with its importance rationale and supporting citations."""
import argparse
import json
import os
from pathlib import Path
import re


def render(task_dir: Path, footer: str = "") -> str:
    metadata = json.loads((task_dir / "task.json").read_text())
    readme = (task_dir / "README.md").read_text()
    paragraphs = [f"**{metadata['title']}**", f"Instruction: {metadata['instruction']}"]
    if metadata.get("context", "").strip():
        paragraphs.append(metadata["context"].strip())
    heading = re.search(
        r"(?im)^##[ \t]+(?:Importance(?: and sources)?|Why this task matters)[ \t]*$",
        readme,
    )
    if heading:
        remainder = readme[heading.end():]
        next_heading = re.search(r"(?m)^#{1,2}[ \t]+", remainder)
        evidence = remainder[:next_heading.start()] if next_heading else remainder
        if evidence.strip():
            paragraphs.append("Importance and sources:\n\n" + evidence.strip())
    paragraphs.extend([
        f"Scores: {metadata['scores']}",
        f"One task folder: `tasks/{task_dir.name}/`.",
    ])
    if footer.strip():
        paragraphs.append(footer.strip())
    return "\n\n".join(paragraphs) + "\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(render(args.task_dir, os.environ.get("PR_FOOTER", "")))
