#!/usr/bin/env python3
"""Prepare the verified GUI handoff and STEP SDK patch outside the repository.

This does not build, install packages, launch a GUI, or access robot hardware.
"""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ARCHIVE_SHA256 = "0f90c8d1f04d5fbe6a78c22b602d0a44ed23327555003e9e140eb6038be518bb"
HANDOFF_NAME = "gui_sdk_20260930_135448"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    destination = args.output.resolve()
    if destination.exists():
        parser.error("output already exists; use a new directory")
    if hashlib.sha256(args.archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        parser.error("archive does not match the verified 2026-09-30 handoff")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="step-sdk-", dir=destination.parent) as temporary:
        staging = Path(temporary)
        with tarfile.open(args.archive) as archive:
            for member in archive.getmembers():
                target = (staging / member.name).resolve()
                if not target.is_relative_to(staging) or not (member.isfile() or member.isdir()):
                    raise ValueError(f"unsafe archive member: {member.name}")
            archive.extractall(staging)
        handoff = staging / HANDOFF_NAME
        for line in (handoff / "SHA256SUMS").read_text().splitlines():
            expected, name = line.split(maxsplit=1)
            source = (handoff / name.lstrip("*")).resolve()
            if not source.is_relative_to(handoff):
                raise ValueError(f"unsafe checksum path: {name}")
            if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise ValueError(f"checksum mismatch: {name}")
        patch = Path(__file__).with_name("sdk_gui_compat.patch").resolve()
        subprocess.run(
            ["patch", "--batch", "--forward", "-p1", "-i", str(patch)],
            cwd=handoff / "cpp_sdk", check=True,
        )
        shutil.move(str(handoff), str(destination))
    print(f"Prepared {destination}; no hardware accessed or runtime defaults changed")


if __name__ == "__main__":
    main()
