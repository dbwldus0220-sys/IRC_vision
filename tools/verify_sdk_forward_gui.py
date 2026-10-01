#!/usr/bin/env python3
"""Compare the active C++ playback loop with the supplied GUI without motor I/O."""

import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

from prepare_gui_sdk import ARCHIVE_SHA256


PROBE = r'''
#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
std::int64_t audit_ms = 0;
int audit_read_delay = 0;
std::chrono::steady_clock::time_point auditClockNow() {
    return std::chrono::steady_clock::time_point(
        std::chrono::milliseconds(audit_ms + 1000));
}
int main(int argc, char** argv) {
    auto library = irc_step::MotionLibrary::loadGuiJson(argv[1]);
    auto initial = library.motion(argv[2]).frames().back().angles;
    if (std::string(argv[4]) == "synthetic")
        for (int j = 0; j < 23; ++j) initial[j] = j * .25 - 3;
    audit_read_delay = std::stoi(argv[5]);
    std::vector<int> periods;
    std::istringstream stream(argv[3]);
    std::string token;
    while (std::getline(stream, token, ',')) periods.push_back(std::stoi(token));
    irc_step::MockMotionHardware hw;
    hw.setPresentPositions(initial);
    irc_step::RobotMotionPlayer p(argv[1], hw);
    if (!p.initialize() || p.start(argv[2]) != irc_step::StartResult::Accepted)
        return 2;
    std::cout << std::setprecision(17);
    for (int tick = 0; audit_ms < 20000; ++tick) {
        p.update();
        std::cout << "TRACE\t" << audit_ms;
        for (int j = 0; j < 23; ++j)
            std::cout << '\t' << hw.commandedPositions().at(j);
        std::cout << '\n';
        if (!p.running()) return p.succeeded() ? 0 : 3;
        audit_ms += periods[tick % periods.size()];
    }
    return 4;
}
'''


def prepare_probe(sdk, work):
    """Replace only the clock and one fake read's latency in temporary copies."""
    source = (sdk / "robot_motion_player.cpp").read_text()
    source = source.replace(
        "namespace irc_step {",
        "extern std::chrono::steady_clock::time_point auditClockNow();\n"
        "namespace irc_step {", 1,
    ).replace("Clock::now()", "auditClockNow()")
    (work / "player.cpp").write_text(source)
    mock = (sdk / "mock_motion_hardware.hpp").read_text().replace(
        "namespace irc_step {",
        "extern std::int64_t audit_ms;\nextern int audit_read_delay;\n"
        "namespace irc_step {", 1,
    ).replace(
        "++read_count_;",
        "++read_count_; audit_ms += audit_read_delay; audit_read_delay = 0;", 1,
    )
    (work / "mock_motion_hardware.hpp").write_text(mock)
    (work / "trace.cpp").write_text(PROBE)
    binary = work / "trace"
    subprocess.run([
        "g++", "-std=c++20", "-I", str(sdk), str(work / "trace.cpp"),
        str(work / "player.cpp"), str(sdk / "motion_pattern.cpp"),
        "-o", str(binary),
    ], check=True)
    return binary


def load_gui(archive_path, work):
    """Use verified GUI methods with the supplier's fake I/O harness."""
    if hashlib.sha256(archive_path.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise ValueError("GUI archive differs from the verified handoff")
    with tarfile.open(archive_path) as archive:
        for name in ("sdk_gui.py", "test_sdk_gui_timed_playback.py"):
            member = next(n for n in archive.getnames() if n.endswith("/gui/" + name))
            (work / name).write_bytes(archive.extractfile(member).read())
    spec = importlib.util.spec_from_file_location(
        "gui_reference", work / "test_sdk_gui_timed_playback.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compare(gui, binary, catalog, motion, periods, pose, read_delay):
    player = gui.player()
    gui.set_frames(player, motion["frames"])
    player.online_joints = set(range(23))
    player.start_angles = {
        int(k): v for k, v in motion["frames"][-1]["angles"].items()
    } if pose == "last" else {j: j * .25 - 3 for j in range(23)}
    player.joints = player.start_angles.copy()
    player.playback_speed = motion["playback_speed"]
    player.playback_repeat_target = motion["repeat_count"]
    player.anim_start_time = read_delay / 1000
    expected = {}
    ms, tick = read_delay, 0
    with contextlib.redirect_stdout(io.StringIO()):
        while ms < 20000:
            gui.tick(player, ms / 1000)
            expected[ms] = player.calls[-1][1].copy()
            if not player.is_playing:
                break
            ms += periods[tick % len(periods)]
            tick += 1
    if player.is_playing:
        raise AssertionError("GUI did not finish within the fake time budget")
    output = subprocess.check_output([
        str(binary), str(catalog), motion["name"],
        ",".join(map(str, periods)), pose, str(read_delay),
    ], text=True)
    actual = {}
    for line in output.splitlines():
        parts = line.split("\t")
        if parts[0] == "TRACE":
            actual[int(parts[1])] = dict(enumerate(map(float, parts[2:])))
    if actual.keys() != expected.keys():
        raise AssertionError((motion["name"], periods, pose, read_delay,
                              "timing differs", max(actual), max(expected)))
    worst = max(abs(actual[t][j] - expected[t][j])
                for t in expected for j in range(23))
    if worst > 1e-9:
        raise AssertionError((motion["name"], periods, pose, read_delay, worst))
    return {"motion": motion["name"], "periods_ms": periods,
            "initial_pose": pose, "initial_read_delay_ms": read_delay,
            "finish_ms": max(actual), "samples": len(actual),
            "max_goal_difference_deg": worst}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", required=True, type=Path)
    parser.add_argument("--gui-archive", required=True, type=Path)
    parser.add_argument("--catalog", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    motions = [m for m in json.loads(args.catalog.read_text())["motions"]
               if "전진" in m["name"]]
    if not motions:
        raise ValueError("No forward motions in catalog")
    with tempfile.TemporaryDirectory(prefix="step_forward_parity_") as directory:
        work = Path(directory)
        gui = load_gui(args.gui_archive, work)
        binary = prepare_probe(args.sdk.resolve(), work)
        results = [compare(gui, binary, args.catalog.resolve(), m, periods, pose, delay)
                   for m in motions for periods in ([5], [10], [20], [5, 7, 3, 11])
                   for pose in ("last", "synthetic") for delay in (0, 30)]
    if args.output:
        args.output.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n")
    print(f"PASS: {len(results)} forward playback cases, "
          f"{sum(r['samples'] for r in results)} timestamps, all 23 joint goals; "
          "fake I/O only")


if __name__ == "__main__":
    main()
