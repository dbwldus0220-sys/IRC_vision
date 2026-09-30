"""Export catalog review subsets without changing operational motion data."""

import ast
import csv
import hashlib
import json
from pathlib import Path

import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
RUNTIME = ROOT / "artifacts/robot_motions_runtime.json"
REFERENCE = Path("/home/jet/Downloads/robot_motions(35).json")
ALIASES = ROOT / "src/irc_step_motion_executor/config/motion_aliases.yaml"
BRIDGE = ROOT / "src/mission_control/mission_control/motion_command_bridge_node.py"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for key, item in value.items():
            yield from strings(key)
            yield from strings(item)
    elif isinstance(value, (list, tuple, set, frozenset)):
        for item in value:
            yield from strings(item)


def write_json(name, data):
    (OUT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")


hashes = {str(path): digest(path) for path in (RUNTIME, REFERENCE, ALIASES, BRIDGE)}
runtime = json.loads(RUNTIME.read_text())
reference = json.loads(REFERENCE.read_text())
aliases = yaml.safe_load(ALIASES.read_text())["motion_aliases"]
runtime_names = {motion["name"] for motion in runtime["motions"]}
reference_names = {motion["name"] for motion in reference["motions"]}
assert len(runtime_names) == len(runtime["motions"])
assert len(reference_names) == len(reference["motions"])
assert set(aliases.values()) <= runtime_names

# Expand class dictionaries/comprehensions, but exclude constants never read by
# any bridge method. Literal mentions are conservative evidence of a connection,
# not proof that a particular mission execution will choose that branch.
tree = ast.parse(BRIDGE.read_text())
bridge_class = next(node for node in tree.body if isinstance(node, ast.ClassDef))
method_nodes = [
    child for method in bridge_class.body if isinstance(method, ast.FunctionDef)
    for child in ast.walk(method)
]
referenced_strings = {
    node.value for node in method_nodes
    if isinstance(node, ast.Constant) and isinstance(node.value, str)
}
referenced_attributes = {
    node.attr for node in method_nodes
    if isinstance(node, ast.Attribute) and node.attr.isupper()
}
for attribute in referenced_attributes:
    referenced_strings.update(strings(getattr(MotionCommandBridgeNode, attribute, None)))
connected_aliases = set(aliases) & referenced_strings
connected_names = {aliases[alias] for alias in connected_aliases}
runtime_only = runtime_names - reference_names
unused_auto = runtime_names - connected_names
union = runtime_only | unused_auto

rows = []
for motion in runtime["motions"]:
    name = motion["name"]
    motion_aliases = sorted(key for key, value in aliases.items() if value == name)
    startup_frames = sum(frame.get("name") == "오뒤412" for frame in motion["frames"])
    rows.append({
        "name": name,
        "in_latest_json": name in reference_names,
        "in_runtime_only": name in runtime_only,
        "auto_command_connected": name in connected_names,
        "no_alias": not motion_aliases,
        "aliases": motion_aliases,
        "connected_aliases": sorted(set(motion_aliases) & connected_aliases),
        "startup_pose_frame_count": startup_frames,
        "frame_count": len(motion["frames"]),
        "max_seq_ms": motion["max_seq_ms"],
    })

for filename, names in (
    ("runtime_only_motions.json", runtime_only),
    ("unused_auto_motions.json", unused_auto),
    ("all_review_motions.json", union),
):
    catalog = {**runtime, "motions": [m for m in runtime["motions"] if m["name"] in names]}
    write_json(filename, catalog)
    extracted = json.loads((OUT / filename).read_text())
    assert len(extracted["motions"]) == len(names)
    originals = {motion["name"]: motion for motion in runtime["motions"]}
    assert all(motion == originals[motion["name"]] for motion in extracted["motions"])

with (OUT / "all_runtime_audit.csv").open("w", encoding="utf-8-sig", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows({**row, "aliases": "; ".join(row["aliases"]),
                     "connected_aliases": "; ".join(row["connected_aliases"])} for row in rows)

manifest = {
    "reference": str(REFERENCE),
    "runtime": str(RUNTIME),
    "sha256": hashes,
    "comparison": "Exact motion names, preserving spaces and punctuation",
    "usage_scope": "Current motion_command_bridge_node action mappings, sequences and preparation branches; no robot execution",
    "runtime_count": len(runtime_names),
    "reference_count": len(reference_names),
    "runtime_only_count": len(runtime_only),
    "unused_auto_count": len(unused_auto),
    "union_count": len(union),
    "unused_aliases": sorted(set(aliases) - connected_aliases),
    "source_only_names": sorted(reference_names - runtime_names),
    "motions": rows,
}
write_json("manifest.json", manifest)

lines = [
    "# 런타임 모션 추출 결과", "",
    f"- 비교 기준: `{REFERENCE}` — {len(reference_names)}개.",
    f"- 현재 런타임: `{RUNTIME}` — {len(runtime_names)}개.",
    f"- 최신 파일에 없는 런타임 모션: **{len(runtime_only)}개**.",
    f"- 자동 명령 생성 경로에서 미사용인 모션: **{len(unused_auto)}개**.",
    f"- 위 두 조건의 합집합: **{len(union)}개** (중복 {len(runtime_only & unused_auto)}개).", "",
    "## 추출 파일", "",
    "- [최신 JSON에 없는 10개](runtime_only_motions.json)",
    "- [자동 명령 경로 미사용 20개](unused_auto_motions.json)",
    "- [두 조건 합집합 21개](all_review_motions.json)",
    "- [런타임 89개 전체 판정 CSV](all_runtime_audit.csv)",
    "- [입력 SHA-256 및 상세 판정](manifest.json)", "",
    "모션명은 공백·괄호까지 그대로 비교했다. 추출 JSON은 런타임의 해당 모션을 필드 변경 없이 복사했다.",
    "운영 런타임, 최신 원본, alias, 제어 코드는 변경하지 않았다.", "",
    "## 판정 범위", "",
    "‘자동 미사용’은 현재 bridge의 액션 매핑·복합 시퀀스·준비 동작에서 해당 모션으로 연결되는 경로를 찾지 못했다는 뜻이다.",
    "alias가 남아 있으면 수동 executor 요청으로 실행할 수 있으며, 실제 주행 로그의 미실행 여부와는 다르다.",
    "호환 경로를 포함해 코드 연결이 하나라도 있으면 보수적으로 사용 연결 있음으로 분류했다.",
    "다른 alias가 미사용이어도 같은 모션에 사용 중인 alias가 있으면 모션 전체를 미사용으로 분류하지 않았다.", "",
    "시작 자세 로더는 모션 실행과 별개로 모든 모션의 `오뒤412` 프레임을 검색한다.",
    "아래 시작 자세 열이 있는 모션은 자동 재생 경로가 없어도 시작 자세 검사에 간접 사용된다. 삭제 승인 목록이 아니다.", "",
    "## 전체 추출 대상", "",
    "| 모션명 | 최신 JSON | 자동 명령 연결 | alias | 오뒤412 프레임 수 |",
    "|---|---|---|---|---:|",
]
for row in rows:
    if row["name"] in union:
        lines.append(
            f"| {row['name']} | {'있음' if row['in_latest_json'] else '**없음**'} "
            f"| {'있음' if row['auto_command_connected'] else '없음'} "
            f"| {', '.join(row['aliases']) or '없음'} | {row['startup_pose_frame_count']} |"
        )
lines += [
    "", "## 개별 확인 사항", "",
    "- `찐미세오뒤에서 오뒤(골대 카메라90도)`는 최신 JSON에는 없지만 `fine_to_turn_ready_90`으로 사용된다. bridge의 `_turn_prepare_motion()`이 골대 카메라 자세에서 미세 전진 후 제자리 회전 전에 선택한다.",
    "- `찐미세오뒤에서기본자세(골대 카메라90도)`는 alias와 `GOAL_FINE_CRAB_PREPARE_MOTION_ID` 선언만 남아 있다. 실제 오른쪽 꽃게 준비는 `GOAL_FINE_RIGHT_CRAB_PREPARE_MOTION_ID`를 사용한다.",
    "- `찐찐라인복귀좌회전45도(6회)`는 executor의 카메라 덮어쓰기 대상 목록에도 있지만, 이 목록은 실행 명령을 생성하지 않는다. 현재 bridge의 왼쪽 복귀 매핑은 4회만 생성한다.",
    "- `찐오뒤에서기본자세(골대 카메라90도)`와 `찐찐전진45(8회)`는 alias는 있지만 현재 bridge에서 생성하는 요청 경로가 없다.",
    "- `찐오뒤412`, `찐찐라인복귀좌회전45도(6회)`, `찐찐전진45(8회)`는 `오뒤412` 프레임을 포함하므로 시작 자세 검사에는 참여한다.", "",
    "## 반대로 최신 JSON에만 있는 모션", "",
]
lines += [f"- {name}" for name in sorted(reference_names - runtime_names)]
lines += ["", "## 검증", "",
          "- 양쪽 모션명 중복 없음, 모든 alias 대상이 런타임에 존재함.",
          "- 추출 개수 및 모든 추출 모션의 전체 필드가 런타임 원본과 일치함.",
          "- 입력 네 파일의 SHA-256이 추출 전후 동일함.",
          "- 로봇 실행 없이 코드·설정·JSON만 검사함.", ""]
(OUT / "README.md").write_text("\n".join(lines))
assert all(digest(Path(path)) == value for path, value in hashes.items())
print(json.dumps({key: manifest[key] for key in (
    "runtime_count", "reference_count", "runtime_only_count", "unused_auto_count", "union_count",
)}, ensure_ascii=False))
print("All extracted motion fields match runtime; input hashes unchanged.")
