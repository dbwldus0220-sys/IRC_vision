#!/usr/bin/env python3
"""Run the hardware-free executable with checked PC bytes and isolated fixtures."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile

binary, runtime, pc = sys.argv[1:]
assert hashlib.sha256(Path(pc).read_bytes()).hexdigest() == (
    "b4d228b865cef7f582bdb67f1fe5d20be4c82178a2553810a30fba2e48fd811e"
)
with tempfile.TemporaryDirectory(prefix="step-common-playback-") as temporary:
    subprocess.run([binary, runtime, pc, temporary], check=True)
