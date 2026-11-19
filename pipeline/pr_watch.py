#!/usr/bin/env python3
"""Read our PRs, including newly closed ones; keep local review evidence only."""
import argparse
import concurrent.futures
import datetime as dt
import fcntl
import json
import pathlib
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parent
def gh(*args):
    return json.loads(subprocess.check_output(["gh", *args], text=True, timeout=90))

def check():
    prs = gh("pr", "list", "--repo", "robocurve/benchmark-tasks", "--author",
             "Tian-James-Pu", "--state", "all", "--limit", "200", "--json", "number")
    numbers = {p["number"] for p in prs}
    for line in (ROOT / "submitted.tsv").read_text().splitlines():
        if line.strip():
            numbers.add(int(line.split("\t")[-1].rstrip("/").split("/")[-1]))
    def fetch(n):
        return gh("pr", "view", str(n), "--repo", "robocurve/benchmark-tasks",
                  "--json", "number,title,url,state,mergedAt,labels,comments,author,headRefName")
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(fetch, sorted(numbers)))
    now = dt.datetime.now(dt.timezone.utc)
    snapshot = {"checked_at": now.isoformat(),
                "next_check_at": (now + dt.timedelta(hours=2)).isoformat(), "prs": rows}
    dest = ROOT / "pr-watch-latest.json"
    old = json.loads(dest.read_text()) if dest.exists() else {"prs": []}
    def signature(p):
        return p["state"], tuple(c["id"] for c in p["comments"]), tuple(x["name"] for x in p["labels"])
    prior = {p["number"]: signature(p) for p in old["prs"]}
    changed = [p for p in rows if prior.get(p["number"]) != signature(p)]
    temporary = dest.with_suffix(".tmp")
    temporary.write_text(json.dumps(snapshot, indent=2) + "\n")
    temporary.replace(dest)
    if changed:
        with (ROOT / "pr-watch-changes.jsonl").open("a") as out:
            out.write(json.dumps({"checked_at": now.isoformat(), "prs": changed}) + "\n")
    print(now.isoformat(), "checked", len(rows), "PRs; changed", [p["number"] for p in changed], flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    with (ROOT / "pr-watch.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("PR watcher already running")
        while True:
            try:
                check()
                delay = 7200
            except Exception as exc:
                print(dt.datetime.now(dt.timezone.utc).isoformat(), "check failed:", str(exc), flush=True)
                if args.once:
                    raise
                delay = 300
            if args.once:
                return
            time.sleep(delay)

if __name__ == "__main__":
    main()
