#!/usr/bin/env python3
"""Apply the reviewed SDK delta only to its exact, hash-verified base.

This edits source files only. It never builds or initializes hardware.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / "sdk_common_playback_manifest.json").read_text())
    current = {name: digest(args.sdk / name) for name in manifest["files"]}
    if all(current[n] == v["after"] for n, v in manifest["files"].items()):
        print("SDK already matches the reviewed common playback patch")
        return
    if all(current[n] == v["before"] for n, v in manifest["files"].items()):
        patch = "sdk_common_playback.patch"
    elif current == manifest.get("previous_files"):
        patch = manifest["upgrade_patch"]
    else:
        raise SystemExit("Refusing to overwrite SDK sources outside the verified original/previous revisions")
    command = ["patch", "--batch", "--forward", "-p1", "-i", str(root / patch)]
    subprocess.run(command + ["--dry-run"], cwd=args.sdk, check=True)
    if args.apply:
        subprocess.run(command, cwd=args.sdk, check=True)
        for name, value in manifest["files"].items():
            if digest(args.sdk / name) != value["after"]:
                raise SystemExit(f"Patched SDK hash mismatch: {name}")
    else:
        print("Verified SDK base and patch; no source files changed")


if __name__ == "__main__":
    main()
