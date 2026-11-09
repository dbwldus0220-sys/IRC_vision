"""Check the saved decision code/catalog without reverting live workspace files."""

import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch

import pytest


UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
name = 'mission_control.motion_decision_node'
spec = importlib.util.spec_from_file_location(
    name, UPDATE / 'before/src/mission_control/mission_control/motion_decision_node.py')
module = importlib.util.module_from_spec(spec)
sys.modules[name] = module
spec.loader.exec_module(module)
import mission_control
mission_control.motion_decision_node = module

read_text = Path.read_text
catalogs = {ROOT / f'artifacts/robot_motions_{kind}.json':
            UPDATE / f'before/artifacts/robot_motions_{kind}.json'
            for kind in ('pc', 'runtime')}


def read_before(path, *args, **kwargs):
    return read_text(catalogs.get(path.resolve(), path), *args, **kwargs)


with patch.object(Path, 'read_text', read_before):
    raise SystemExit(pytest.main([
        '-q',
        *[str(ROOT / f'src/mission_control/test/test_{test}.py') for test in (
            'line_recovery_late_queue', 'post_shot_forward_prequeue',
            'object_walk_prequeue', 'motion_command_gate', 'executor_heartbeat_watchdog')],
        f'--junitxml={UPDATE}/queue_release_baseline.xml',
    ]))
