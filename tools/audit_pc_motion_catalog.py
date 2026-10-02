#!/usr/bin/env python3
"""Compare immutable PC and production catalogs without normalizing either."""
import argparse
import ast
import difflib
import hashlib
import importlib
import json
from pathlib import Path

import yaml

PC_SHA256 = "b4d228b865cef7f582bdb67f1fe5d20be4c82178a2553810a30fba2e48fd811e"
POLICY_IDS = {"0", "4", "5"}


def differences(pc, jetson):
    return {key: {"pc": pc.get(key), "jetson": jetson.get(key),
                  "pc_present": key in pc, "jetson_present": key in jetson}
            for key in sorted(pc.keys() | jetson.keys()) if pc.get(key) != jetson.get(key)
            or (key in pc) != (key in jetson)}


def compare(pc_path, jetson_path, aliases_path, source_tree, bridge_module=None):
    raw = pc_path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != PC_SHA256:
        raise ValueError("PC catalog SHA256 mismatch; refusing an unverified baseline")
    pc = {m["name"]: m for m in json.loads(raw)["motions"]}
    jetson = {m["name"]: m for m in json.loads(jetson_path.read_bytes())["motions"]}
    aliases = yaml.safe_load(aliases_path.read_text())["motion_aliases"]
    report = {
        "pc_sha256": PC_SHA256,
        "jetson_sha256": hashlib.sha256(jetson_path.read_bytes()).hexdigest(),
        "pc_counts": [len(pc), sum(len(m["frames"]) for m in pc.values())],
        "jetson_counts": [len(jetson), sum(len(m["frames"]) for m in jetson.values())],
        "pc_only": sorted(pc.keys() - jetson.keys()),
        "jetson_only": sorted(jetson.keys() - pc.keys()),
        "aliases_missing_from_pc": {a: n for a, n in aliases.items() if n not in pc},
        "aliases_missing_from_jetson": {a: n for a, n in aliases.items() if n not in jetson},
        "frame_count_changes": [], "motions": {}, "alias_references": {},
        "alias_targets": aliases,
    }
    if bridge_module:
        # Importing the class does not instantiate a node or access hardware.
        bridge = importlib.import_module(bridge_module).MotionCommandBridgeNode
        maps = {}
        for key, value in vars(bridge).items():
            if not key.isupper() or not ("MOTION" in key or "SEQUENCE" in key):
                continue
            items = value.values() if isinstance(value, dict) else (
                value if isinstance(value, (tuple, list, set)) else [value])
            maps[key] = sorted({v for v in items if isinstance(v, str) and not v.startswith("__")})
        report["bridge_motion_constants"] = maps
        report["bridge_actions_missing_aliases"] = {
            a: n for a, n in bridge.ACTION_TO_MOTION_ID.items() if n not in aliases}
        report["bridge_constants_missing_aliases"] = {
            k: [v for v in values if v not in aliases]
            for k, values in maps.items() if any(v not in aliases for v in values)}
    for name in sorted(pc.keys() | jetson.keys()):
        a, b = pc.get(name), jetson.get(name)
        af, bf = a["frames"] if a else [], b["frames"] if b else []
        if len(af) != len(bf):
            report["frame_count_changes"].append({
                "motion": name, "pc": len(af), "jetson": len(bf),
                "delta_jetson_minus_pc": len(bf) - len(af)})
        entry = {"metadata": differences(
            {k: v for k, v in (a or {}).items() if k != "frames"},
            {k: v for k, v in (b or {}).items() if k != "frames"}),
            "aligned_frames": [], "insertions_deletions_replacements": []}
        matcher = difflib.SequenceMatcher(a=[f["name"] for f in af],
                                         b=[f["name"] for f in bf], autojunk=False)
        for tag, i, end_i, j, end_j in matcher.get_opcodes():
            if tag != "equal":
                entry["insertions_deletions_replacements"].append({
                    "operation": tag, "pc_indices": [i, end_i], "jetson_indices": [j, end_j],
                    "pc_frames": af[i:end_i], "jetson_frames": bf[j:end_j]})
                continue
            for pi, ji in zip(range(i, end_i), range(j, end_j)):
                x, y = af[pi], bf[ji]
                angles = differences(x["angles"], y["angles"])
                fields = differences({k: v for k, v in x.items() if k != "angles"},
                                     {k: v for k, v in y.items() if k != "angles"})
                if angles or fields:
                    entry["aligned_frames"].append({
                        "pc_index": pi, "jetson_index": ji, "name": x["name"],
                        "policy_angles": {k: v for k, v in angles.items() if k in POLICY_IDS},
                        "other_angles": {k: v for k, v in angles.items() if k not in POLICY_IDS},
                        "fields": fields})
        report["motions"][name] = entry
    for path in sorted(source_tree.rglob("*.py")):
        if "test" in path.parts or path.name.startswith("test_"): continue
        try: tree = ast.parse(path.read_text())
        except (SyntaxError, UnicodeError): continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str) and node.value in aliases:
                report["alias_references"].setdefault(node.value, []).append(f"{path}:{node.lineno}")
    report["net_frame_delta"] = sum(x["delta_jetson_minus_pc"] for x in report["frame_count_changes"])
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pc", type=Path, required=True)
    parser.add_argument("--jetson", type=Path, required=True)
    parser.add_argument("--aliases", type=Path, required=True)
    parser.add_argument("--source-tree", type=Path, default=Path("src"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bridge-module", help="Optional installed ROS bridge module; no node is created")
    args = parser.parse_args()
    report = compare(args.pc, args.jetson, args.aliases, args.source_tree, args.bridge_module)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("pc_counts", "jetson_counts", "pc_only", "jetson_only",
        "aliases_missing_from_pc", "frame_count_changes", "net_frame_delta")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
