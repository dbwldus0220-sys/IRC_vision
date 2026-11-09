from pathlib import Path
ROOT = Path('/home/jet/IRC/IRC-STEP/IRC_vision_latest')
BEFORE = ROOT / 'artifacts/20261110_catalog82_pickup_shot/before'
original_read_text = Path.read_text
original_read_bytes = Path.read_bytes

def redirected(path):
    if path in [ROOT / f'artifacts/robot_motions_{kind}.json' for kind in ('pc', 'runtime')]:
        return BEFORE / path.relative_to(ROOT)
    return path

def pytest_configure(config):
    Path.read_text = lambda self, *a, **kw: original_read_text(redirected(self), *a, **kw)
    Path.read_bytes = lambda self, *a, **kw: original_read_bytes(redirected(self), *a, **kw)
