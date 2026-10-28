#!/usr/bin/env python3
"""Restore only this change's files; optionally restore the saved build/install trees."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

parser = argparse.ArgumentParser()
parser.add_argument('--with-build', action='store_true', help='Also restore pre-change build/install files')
args = parser.parse_args()
backup = Path(__file__).resolve().parent
root = backup.parents[1]
manifest = json.loads((backup/'changes.json').read_text())
# Refuse to discard later edits to files created by this change.
for name in manifest['added']:
    path = root/name
    if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest() != manifest['after_sha256'][name]:
        raise SystemExit(f'New file has later edits; preserve it before restoring: {name}')
with tarfile.open(backup/'before.tar.gz') as archive:
    for name in manifest['modified']:
        extra = backup/'additional_before'/name
        target = root/name
        target.parent.mkdir(parents=True, exist_ok=True)
        if extra.exists():
            target.write_bytes(extra.read_bytes())
        else:
            target.write_bytes(archive.extractfile(name).read())
    if args.with_build:
        for member in archive.getmembers():
            if member.name in ('build', 'install') or member.name.startswith(('build/', 'install/')):
                archive.extract(member, root)
for name in manifest['added']:
    (root/name).unlink(missing_ok=True)
print('Source/settings/motion JSON restored. Evidence and backup retained.')
print('Saved build/install restored.' if args.with_build else 'Rebuild before running; installed binary is still the new version.')
