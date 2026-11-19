"""Check the ambiguous team name in prose, allowing Python geometry identifiers."""
import io
from pathlib import Path
import re
import sys
import tokenize


def hits(path):
    text = path.read_text(errors="replace")
    if path.suffix == ".py":
        try:
            chunks = [(t.start[0], t.string) for t in tokenize.generate_tokens(io.StringIO(text).readline)
                      if t.type in (tokenize.COMMENT, tokenize.STRING)]
        except (tokenize.TokenError, IndentationError):
            chunks = [(1, text)]  # fail conservatively for malformed source
    else:
        chunks = [(1, text)]
    for start, chunk in chunks:
        for offset, line in enumerate(chunk.splitlines()):
            if re.search(r"\bprism\b", line, re.IGNORECASE):
                yield start + offset, line


def main(root):
    found = False
    for path in sorted(Path(root).rglob("*")):
        if not path.is_file():
            continue
        for number, line in hits(path):
            print(f"{path}:{number}:{line}")
            found = True
    if found:
        print("submit: identifying Prism text found above; remove it", file=sys.stderr)
    return int(found)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
