"""Lock the production bridge-to-C++ alias contract to approved entries."""

from pathlib import Path
import copy
import hashlib
import json

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
import pytest
import yaml


ALIAS_PATH = (
    Path(__file__).resolve().parents[2]
    / "irc_step_motion_executor"
    / "config"
    / "motion_aliases.yaml"
)
RUNTIME_CATALOG_PATH = (
    Path(__file__).resolve().parents[3]
    / "artifacts"
    / "robot_motions_runtime.json"
)


@pytest.mark.parametrize("angle,count", [
    (45, 2), (45, 4), (45, 6), (45, 8),
    (90, 4), (90, 6), (90, 8),
])
def test_catalog30_forward_preserves_supplied_motion(angle, count):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    by_name = {motion["name"]: motion for motion in motions}
    expected_name = f"찐전진{angle}({count}회)"
    expected_aliases = {
        (45, 2): ("ball_camera_down_forward_2",),
        (45, 4): ("sdk_forward_4", "line_forward_4", "post_ball_forward_4", "ball_camera_down_forward_4"),
        (45, 6): ("line_forward_6", "post_ball_forward_6"),
        (45, 8): ("post_ball_forward_8",),
        (90, 4): ("goal_camera_90_forward_4",),
        (90, 6): ("goal_camera_90_forward_6",),
        (90, 8): (),
    }
    for alias in expected_aliases[angle, count]:
        assert aliases[alias] == expected_name
    motion = by_name[expected_name]
    # robot_motions(30).json repeats an eight-frame, two-cycle gait.
    assert motion["repeat_count"] == count // 2
    assert len(motion["frames"]) == 8
    assert motion["playback_speed"] == pytest.approx(1.05)
    assert motion["completion"]["position_tolerance_deg"] == 5.0
    # Preserve the entire supplied motion, except the completion tolerance.
    source_path = RUNTIME_CATALOG_PATH.parent / "catalog30_forward_update_backup/source_requested_motions.json"
    source = {m["name"]: m for m in json.loads(source_path.read_text())["motions"]}
    expected = source[expected_name]
    expected["completion"]["position_tolerance_deg"] = 5.0
    assert motion == expected


def test_catalog30_replaces_only_requested_forward_motions_and_aliases():
    manifest = json.loads((RUNTIME_CATALOG_PATH.parent / "catalog30_forward_update_manifest.json").read_text())
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    catalog = {m["name"]: m for m in motions}
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    # The hurdle import and restored close-ball retreat each add one motion.
    assert len(catalog) == len(motions) == manifest["after_count"] + 2
    assert not set(manifest["renames"]) & set(catalog)
    assert set(aliases.values()) <= set(catalog)
    # Later camera updates supersede the historical catalog30 digests.
    camera_update_dir = RUNTIME_CATALOG_PATH.parent / "20260929_camera_motion_update"
    camera_update = json.loads((camera_update_dir / "comparison.json").read_text())
    camera_source = {
        m["name"]: m for m in
        json.loads((camera_update_dir / "source_robot_motions.json").read_text())["motions"]
    }
    camera_names = set(camera_update["replaced_motion_names"])
    assert len(camera_names) == 16
    for name in camera_names:
        expected = copy.deepcopy(camera_source[name])
        expected["completion"]["position_tolerance_deg"] = 5.0
        assert catalog[name] == expected
        assert all(frame["angles"]["0"] == -64.0 for frame in catalog[name]["frames"])
    for name, digest in manifest["unchanged_sha256"].items():
        # The latest shot source is checked by the dedicated goal-shot test.
        if name in camera_names or name == "찐골넣기":
            continue
        motion = catalog[name]
        original_counts = {
            "찐후진하고 제자리우회전(공)": 7,
            "찐후진에서 제자리좌회전(공)": 4,
        }
        if name in original_counts:
            motion = dict(motion)
            motion["frames"] = motion["frames"][:2 + 2 * original_counts[name]]
            last = motion["frames"][-1]
            motion["max_seq_ms"] = last["start_ms"] + last["time_ms"]
        encoded = json.dumps(motion, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        assert hashlib.sha256(encoded.encode()).hexdigest() == digest
    before_path = RUNTIME_CATALOG_PATH.parent / "catalog30_forward_update_backup/motion_aliases.yaml"
    expected_aliases = yaml.safe_load(before_path.read_text())["motion_aliases"]
    for alias, change in manifest["changed_aliases"].items():
        expected_aliases[alias] = change["after"]
    expected_aliases["hurdle"] = "찐허들"
    expected_aliases["pickup_lost_ball_backward_1"] = "후진실전-2(1회, 픽업 카메라0도)"
    assert aliases == expected_aliases


def test_post_ball_transition_contains_only_camera_without_stationary_pause():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    assert MotionCommandBridgeNode.POST_BALL_GOAL_TRANSITION_SEQUENCE == (
        "post_ball_camera_90", MotionCommandBridgeNode.POST_BALL_CAMERA_DWELL_MARKER,
    )
    assert MotionCommandBridgeNode.POST_BALL_CAMERA_PAUSE_SEC == 0.0
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID["POST_BALL_GOAL_TRANSITION"] == "post_ball_camera_90"
    assert motions[aliases["post_ball_camera_90"]]["frames"]
    assert aliases["line_forward_6"] == "찐전진45(6회)"


def test_goal_shot_matches_supplied_motion_with_five_degree_tolerance():
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text(encoding="utf-8"))["motions"]
    target = next(motion for motion in motions if motion["name"] == "찐골넣기")
    source_path = RUNTIME_CATALOG_PATH.parent / "20260929_goal_shot_update/source_shot.json"
    expected = json.loads(source_path.read_text(encoding="utf-8"))
    expected["completion"]["position_tolerance_deg"] = 5.0
    assert target == expected


def test_post_shot_fixed_turns_and_forward_resolve_to_runtime_motions():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    expected = {
        "POST_SHOT_TURN_RIGHT_9": ("찐기본자세에서 제자리 우회전(골대)", 1),
        "POST_SHOT_TURN_LEFT_4": ("찐기본자세에서 제자리좌회전(골대)", 1),
        "POST_SHOT_FORWARD": ("찐전진45(6회)", 3),
    }
    for action, (name, count) in expected.items():
        motion_id = MotionCommandBridgeNode.motion_id_for_action(action)
        assert aliases[motion_id] == name
        assert motions[name]["repeat_count"] == count


def test_updated_forward_and_stationary_right_turn_catalog():
    aliases = yaml.safe_load(ALIAS_PATH.read_text(encoding="utf-8"))[
        "motion_aliases"
    ]
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text(encoding="utf-8"))[
        "motions"
    ]
    by_name = {motion["name"]: motion for motion in motions}
    assert len(by_name) == len(motions)

    for prefix in ("전진45도-1", "전진0도-1", "전진90도-1"):
        for count in (2, 4, 6, 8, 10):
            motion = by_name[f"{prefix}({count}회)"]
            assert motion["repeat_count"] == count
            assert len(motion["frames"]) == 4

    for alias_prefix, angle in (
        ("post_ball_line_turn_right", 45),
        ("pickup_camera_down_turn_right", 0),
        ("goal_camera_90_turn_right", 90),
    ):
        for count in (2, 3, 5, 7, 9):
            name = ("찐제자리우회전45도(9회)" if angle == 45 and count == 9
                    else f"찐제자리우회전{angle}도-1({count}회)")
            assert aliases[f"{alias_prefix}_{count}"] == name
            motion = by_name[name]
            assert motion["repeat_count"] == count
            assert len(motion["frames"]) == 2
            assert motion["playback_speed"] == 0.9
            assert motion["end_pose"] == (
                "오뒤415" if angle == 45 else f"오뒤415({angle}도)"
            )
            assert motion["frames"] == by_name[
                f"찐제자리우회전{angle}도-1(2회)"
            ]["frames"]

    assert aliases["stationary_turn_right"] == "찐제자리우회전45도(9회)"
    assert aliases["pickup_first_turn_right_7"] == "찐제자리우회전45도-1(7회)"
    assert aliases["pickup_first_turn_right_9"] == "찐제자리우회전45도(9회)"


def test_production_alias_catalog_contains_only_approved_aliases():
    payload = yaml.safe_load(ALIAS_PATH.read_text(encoding="utf-8"))
    assert payload == {
        "motion_aliases": {
            "sdk_pickup": "찐공잡기리그랩까지 실전",
            "sdk_hurdle": "허들실실전",
            "sdk_forward_4": "찐전진45(4회)",
            "line_forward_2": "전진45도-1(2회)",
            "line_forward_4": "찐전진45(4회)",
            "line_forward_6": "찐전진45(6회)",
            "line_forward_8": "전진45도-1(8회)",
            "line_forward_10": "전진45도-1(10회)",
            "line_turn_left_4": "좌회전실실전(4회)",
            "line_turn_left_6": "좌회전실실전(6회)",
            "line_turn_left_8": "좌회전실실전(8회)",
            "line_turn_left_10": "좌회전실실전(10회)",
            "line_turn_left_12": "좌회전실실전(12회)",
            "line_turn_left_15": "좌회전실실전(15회)",
            "line_turn_right_2": "우회전실실전(4회)",
            "line_turn_right_4": "우회전실실전(6회)",
            "line_turn_right_6": "우회전실실전(8회)",
            "line_turn_right_8": "우회전실실전(10회)",
            "line_turn_right_10": "우회전실실전(12회)",
            "line_turn_right_large": "우회전실실전(15회)",
            **{
                f"line_recovery_left_{count}": f"라인복귀좌회전({count}회)"
                for count in (2, 3, 4, 5, 6, 7, 8, 10, 13)
            },
            **{
                f"line_recovery_right_{count}": f"라인복귀우회전({count}회)"
                for count in (2, 3, 4, 5, 6, 7, 8, 10, 12, 15)
            },
            "line_search_left_2": "찐제자리좌회전45도-1(2회)",
            "line_search_right_5": "찐제자리우회전45도-1(5회)",
            "stationary_turn_left": "찐제자리좌회전45도-1(6회)",
            "stationary_turn_right": "찐제자리우회전45도(9회)",
            "ball_camera_down_forward_2": "전진0도-1(2회)",
            "ball_camera_down_forward_4": "전진0도-1(4회)",
            "ball_camera_down_forward_6": (
                "전진0도-1(6회)"
            ),
            "ball_general_fine_forward_8": "찐미세45도-4",
            "pickup_fine_forward_0": "찐미세0도-4",
            "pickup_fine_prepare": "찐오뒤에서미세오뒤",
            "pickup_crab_prepare": "오뒤에서 기본자세(픽업 카메라0도)",
            "pickup_crab_right_0": "찐미세오옆꽃게0-1(1회)",
            "pickup_fine_to_crab_right_0": "찐미세오뒤에서오옆꽃게0도",
            "pickup_crab_left_0": "찐미세왼옆꽃게0도-1",
            "pickup_lost_ball_backward_1": "후진실전-2(1회, 픽업 카메라0도)",
            "pickup_pre_backward_camera_down": (
                "찐공잡기전후진-2(2회)"
            ),
            "pickup_grasp_check_pose": "찐공확인자세",
            "pickup_retreat_2": "찐후진실전(1회)",
            "pickup_second_backward_turn_left": "찐후진에서 제자리좌회전(공)",
            "pickup_first_backward_turn_right": "찐후진하고 제자리우회전(공)",
            "pickup_first_turn_right_7": "찐제자리우회전45도-1(7회)",
            "pickup_second_turn_left_3": "찐제자리좌회전45도-1(3회)",
            "pickup_first_turn_right_9": "찐제자리우회전45도(9회)",
            "pickup_first_to_right_back_camera_45": (
                "왼뒤에서 오뒤로(카메라45도)"
            ),
            **{
                f"pickup_camera_down_turn_left_{count}": (
                    f"찐제자리좌회전0도-1({count}회)"
                )
                for count in range(1, 7)
            },
            **{
                f"pickup_camera_down_turn_right_{count}": (
                    f"찐제자리우회전0도-1({count}회)"
                )
                for count in range(1, 10)
            },
            "post_ball_forward_4": "찐전진45(4회)",
            "post_ball_forward_6": "찐전진45(6회)",
            "post_shot_default_turn_right": "찐기본자세에서 제자리 우회전(골대)",
            "post_shot_default_turn_left": "찐기본자세에서 제자리좌회전(골대)",
            "post_shot_turn_right_9": "찐제자리우회전45도(9회)",
            "post_ball_forward_8": "찐전진45(8회)",
            "post_ball_camera_90": "찐오뒤카메라90도",
            "goal_camera_90_forward_2": "전진90도-1(2회)",
            "goal_camera_90_forward_4": "찐전진90(4회)",
            "goal_camera_90_forward_6": "찐전진90(6회)",
            **{
                f"goal_camera_90_fine_forward_{count}": f"미세90도-4({count}회)"
                for count in range(1, 5)
            },
            "goal_camera_90_crab_right": "찐미세오옆꽃게90-1(1회)",
            "goal_fine_to_crab_right_90": "찐미세오뒤에서 오옆꽃게90도",
            "goal_camera_90_crab_left": "찐미세왼옆꽃게90도-1",
            "goal_forward_to_crab_right_90": "찐오뒤에서 오옆꽃게90도",
            "goal_forward_to_default_90": "찐오뒤에서기본자세(골대 카메라90도)",
            "goal_fine_to_default_90": "찐미세오뒤에서기본자세(골대 카메라90도)",
            **{
                f"post_ball_line_turn_right_{count}": (
                    ("찐제자리우회전45도(9회)" if count == 9
                     else f"찐제자리우회전45도-1({count}회)")
                )
                for count in range(1, 10)
            },
            "post_ball_line_turn_left_1": "찐제자리좌회전45도-1(1회)",
            "post_ball_line_turn_left_2": "찐제자리좌회전45도-1(2회)",
            "post_ball_line_turn_left_3": "찐제자리좌회전45도-1(3회)",
            "post_ball_line_turn_left_4": "찐제자리좌회전45도-1(4회)",
            "post_ball_line_turn_left_5": "찐제자리좌회전45도-1(5회)",
            "post_ball_line_turn_left_6": "찐제자리좌회전45도-1(6회)",
            **{
                f"goal_camera_90_turn_right_{count}": (
                    f"찐제자리우회전90도-1({count}회)"
                )
                for count in (2, 3, 5, 7, 9)
            },
            **{
                f"goal_camera_90_turn_left_{count}": (
                    f"찐제자리좌회전90도-1({count}회)"
                )
                for count in range(1, 7)
            },
            "goal_shot": "찐골넣기",
            "goal_fine_to_default": "찐미세오뒤에서기본자세",
            "sdk_default_to_right_back": "기본에서 오뒤 실실전",
            "pickup": "찐공잡기리그랩까지 실전",
            "hurdle": "허들실실전",
            "forward": "전진45도-1(8회)",
        }
    }


def test_deprecated_left_turn_is_not_a_production_alias_target():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    forbidden = {
        "좌회전실전(9회)",
        "좌회전실전(2회)",
        "좌회전 실전(13회)",
    }
    assert forbidden.isdisjoint(aliases.values())


def test_pickup_does_not_use_intermediate_default_pose():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]

    assert "pickup_left_back_to_default_90" not in aliases
    assert "찐왼뒤에서기본자세(90도)" not in aliases.values()
    assert "찐왼뒤에서기본(0도)" not in aliases.values()
    assert aliases["goal_camera_90_turn_left_1"] == (
        "찐제자리좌회전90도-1(1회)"
    )
    assert aliases["goal_camera_90_turn_left_6"] == (
        "찐제자리좌회전90도-1(6회)"
    )


def test_post_ball_return_motions_resolve_in_runtime_catalog():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        motion["name"]: motion
        for motion in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    for action in (
        "POST_BALL_LINE_TURN_LEFT_1",
        "POST_BALL_LINE_TURN_RIGHT_2",
    ):
        motion_id = MotionCommandBridgeNode.ACTION_TO_MOTION_ID[action]
        assert motions[aliases[motion_id]]["frames"]


def test_every_production_bridge_motion_id_has_an_approved_alias():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    assert set(MotionCommandBridgeNode.ACTION_TO_MOTION_ID.values()) <= set(
        aliases
    )
    pickup_motion_ids = {
        *MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_MOTION_IDS.values(),
        *MotionCommandBridgeNode.PICKUP_MOTION_TAIL,
        *MotionCommandBridgeNode.POST_BALL_GOAL_TRANSITION_SEQUENCE,
        "pickup_second_backward_turn_left",
        "pickup_first_backward_turn_right",
        *MotionCommandBridgeNode.PICKUP_FINE_ALIGN_MOTION_IDS.values(),
    }
    pickup_motion_ids.discard(None)
    pickup_motion_ids.discard(MotionCommandBridgeNode.FINE_ALIGN_MARKER)
    pickup_motion_ids.discard(MotionCommandBridgeNode.POST_BALL_CAMERA_DWELL_MARKER)
    assert pickup_motion_ids <= set(aliases)


def test_counted_turn_aliases_match_family_direction_and_repeat_count():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    catalog = yaml.safe_load(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    motions_by_name = {motion["name"]: motion for motion in catalog}

    expected = {
        "stationary_turn_left": ("찐제자리좌회전45도-1(6회)", 6),
        "stationary_turn_right": ("찐제자리우회전45도(9회)", 9),
        "pickup_first_turn_right_7": ("찐제자리우회전45도-1(7회)", 7),
        "pickup_second_turn_left_3": ("찐제자리좌회전45도-1(3회)", 3),
        "pickup_first_turn_right_9": ("찐제자리우회전45도(9회)", 9),
        **{
            f"post_ball_line_turn_right_{count}": (
                (
                    "찐제자리우회전45도(9회)" if count == 9
                    else f"찐제자리우회전45도-1({count}회)"
                ),
                count,
            )
            for count in (2, 3, 5, 7, 9)
        },
        "post_ball_line_turn_left_1": ("찐제자리좌회전45도-1(1회)", 1),
        "post_ball_line_turn_left_2": ("찐제자리좌회전45도-1(2회)", 2),
        "post_ball_line_turn_left_3": ("찐제자리좌회전45도-1(3회)", 3),
        "post_ball_line_turn_left_4": ("찐제자리좌회전45도-1(4회)", 4),
        "post_ball_line_turn_left_6": ("찐제자리좌회전45도-1(6회)", 6),
        **{
            f"goal_camera_90_turn_right_{count}": (
                f"찐제자리우회전90도-1({count}회)",
                count,
            )
            for count in (2, 3, 5, 7, 9)
        },
        **{
            f"goal_camera_90_turn_left_{count}": (
                f"찐제자리좌회전90도-1({count}회)",
                count,
            )
            for count in range(1, 7)
        },
        **{
            f"pickup_camera_down_turn_left_{count}": (
                f"찐제자리좌회전0도-1({count}회)",
                count,
            )
            for count in range(1, 7)
        },
        **{
            f"pickup_camera_down_turn_right_{count}": (
                f"찐제자리우회전0도-1({count}회)",
                count,
            )
            for count in (2, 3, 5, 7, 9)
        },
    }

    for motion_id, (exact_name, repeat_count) in expected.items():
        assert aliases[motion_id] == exact_name
        assert motions_by_name[exact_name]["repeat_count"] == repeat_count

    assert motions_by_name["찐제자리좌회전90도-1(6회)"][
        "playback_speed"
    ] == pytest.approx(0.9, rel=0.0, abs=1e-12)


def test_all_production_alias_targets_exist_in_catalog():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    catalog = yaml.safe_load(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    motions_by_name = {motion["name"]: motion for motion in catalog}

    assert set(aliases.values()) <= set(motions_by_name)


def test_production_motions_keep_approved_final_tolerances():
    catalog = json.loads(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    motions_by_name = {motion["name"]: motion for motion in catalog}

    for motion in catalog:
        assert motion["completion"]["position_tolerance_deg"] == 5.0

    production_tolerance = motions_by_name[
        "찐공잡기리그랩까지 실전"
    ]["completion"]["position_tolerance_deg"]
    assert 4.75 <= production_tolerance
    assert 5.25 > production_tolerance


def test_latest_pickup_and_grasp_check_motion_definitions_are_mapped():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    motions = yaml.safe_load(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    by_name = {motion["name"]: motion for motion in motions}

    expected = {
        "미세45도-1": (3505, 1, 1.0, "오들401", "미세오뒤401"),
        "미세0도-1": (
            3505,
            1,
            1.0,
            "오들401(0도)",
            "미세오뒤401(0도)",
        ),
        "찐공잡기리그랩까지 실전": (
            8000,
            1,
            1.05,
            "음",
            "기본자세4",
        ),
        "찐공확인자세": (7045, 1, 1.0, "찐공확인자세", "찐공확인자세"),
    }
    for name, metadata in expected.items():
        motion = by_name[name]
        assert (
            motion["max_seq_ms"],
            motion["repeat_count"],
            motion["playback_speed"],
            motion["start_pose"],
            motion["end_pose"],
        ) == metadata

    for motion_id, name in (
        ("ball_general_fine_forward_8", "찐미세45도-4"),
        ("pickup_fine_forward_0", "찐미세0도-4"),
    ):
        assert aliases[motion_id] == name
        motion = by_name[name]
        assert len(motion["frames"]) == 4
        assert motion["repeat_count"] == 5
        assert max(
            frame["start_ms"] + frame["time_ms"]
            for frame in motion["frames"]
        ) == 388

    check_pose = by_name[aliases["pickup_grasp_check_pose"]]
    assert len(check_pose["frames"]) == 1
    assert check_pose["frames"][0]["name"] == "공확인자세"
    assert check_pose["frames"][0]["time_ms"] == 400


def test_top_loss_forward_uses_existing_camera0_two_repeat_motion():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    motion_id = MotionCommandBridgeNode.ACTION_TO_MOTION_ID["BALL_LOST_FORWARD_2"]
    assert motion_id == "ball_camera_down_forward_2"
    assert aliases[motion_id] == "전진0도-1(2회)"
    assert motions[aliases[motion_id]]["repeat_count"] == 2


@pytest.mark.parametrize('count', range(1, 5))
def test_goal_fine_repeats_use_walking_cycle_not_pickup_frames(count):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    motion_id = MotionCommandBridgeNode.ACTION_TO_MOTION_ID[
        f"GOAL_CAMERA90_FINE_FORWARD_{count}"
    ]
    motion = motions[aliases[motion_id]]
    base = motions['찐미세90도-4(1회)']
    assert motion['repeat_count'] == count
    assert len(motion['frames']) == 4
    assert motion['frames'] == base['frames']
    assert motion['start_pose'] == '미세오들4112(90도)'
    assert motion['end_pose'] == '미세오뒤4111(90도)'
    assert motion['playback_speed'] == 1.0
    assert motion['max_seq_ms'] == 3505


@pytest.mark.parametrize('name,digest', [
    # Camera-0 source: artifacts/20260929_camera_motion_update/source_robot_motions.json
    ('찐미세0도-4', '9abdda081d4406738b7e731707ce690d2207ad82287cedc5c0db1fa230d0eb75'),
    ('찐미세45도-4', '7eb93d40e4fea5441dc4b603443bd5cc46f626ee6d4742895912d44dc649e4a0'),
    ('찐미세90도-4(1회)', '05a8bf2428764b71f8bd4cf6b4663227facd5a34cd489de6e977186e86af704f'),
])
def test_updated_fine_frames_match_supplied_walking_data(name, digest):
    motions = {m['name']: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']}
    encoded = json.dumps(motions[name]['frames'], sort_keys=True, separators=(',', ':'))
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


def test_pickup_fine_preparation_resolves_to_requested_single_transition():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        m["name"]: m
        for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    prepare_id = MotionCommandBridgeNode.PICKUP_FINE_PREPARE_MOTION_ID
    assert aliases[prepare_id] == "찐오뒤에서미세오뒤"
    assert motions[aliases[prepare_id]]["repeat_count"] == 1
    assert motions[aliases[prepare_id]]["completion"]["position_tolerance_deg"] == 5.0
    assert aliases["pickup_fine_forward_0"] == "찐미세0도-4"


def test_pickup_retreat_keeps_command_id_with_one_repeat():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']}
    retreat = motions[aliases['pickup_retreat_2']]
    assert aliases['pickup_retreat_2'] == '찐후진실전(1회)'
    assert retreat['repeat_count'] == 1
    assert MotionCommandBridgeNode.PICKUP_MOTION_TAIL[-1] == 'pickup_retreat_2'
    assert MotionCommandBridgeNode.PICKUP_NO_BALL_MOTION_TAIL[-1] == 'pickup_retreat_2'


@pytest.mark.parametrize("motion_id,name,digest", [
    ("pickup", "찐공잡기리그랩까지 실전",
     "49f40834f6b2f9e40a12998766acd6a0dfa91456d3277ccb806fb473a9ecad4b"),
    ("pickup_pre_backward_camera_down", "찐공잡기전후진-2(2회)",
     "7bc8b060ba29ab42314be112a6077132ac3284e3398e45e57afa48a6f3cf7715"),
])
def test_import15_requested_motions_match_source_with_five_degree_tolerance(
    motion_id, name, digest,
):
    """Preserve supplied pickup data, including the 2026-09-29 camera update."""
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        motion["name"]: motion
        for motion in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    assert aliases[motion_id] == name
    motion = motions[name]
    assert motion["completion"]["position_tolerance_deg"] == 5.0
    encoded = json.dumps(
        motion, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    )
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


@pytest.mark.parametrize('line_side', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('direction,korean,counts', [
    ('LEFT', '좌', (2, 3, 4, 5, 6, 7, 8, 10, 13)),
    ('RIGHT', '우', (2, 3, 4, 5, 6, 7, 8, 10, 12, 15)),
])
def test_recovery_turns_resolve_to_same_direction_and_repeat_count(line_side, direction, korean, counts):
    from mission_control.motion_command_gate import normalize_general_action
    aliases = yaml.safe_load(ALIAS_PATH.read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']}
    for count in counts:
        action = f'RECOVER_{line_side}_TURN_{direction}_{count}'
        assert normalize_general_action(action) == action
        motion_id = MotionCommandBridgeNode.ACTION_TO_MOTION_ID[action]
        assert aliases[motion_id] == f'라인복귀{korean}회전({count}회)'
        motion = motions[aliases[motion_id]]
        assert motion['repeat_count'] == count
        assert len(motion['frames']) == 4
        assert motion['playback_speed'] == 0.9
        assert [f['angles'] for f in motion['frames']] == [
            f['angles'] for f in motions[f'라인복귀{korean}회전(4회)']['frames']
        ]


def test_line_search_uses_requested_stationary_motions():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']}
    for action, name, count in [
        ('LINE_LOST_TURN_LEFT', '찐제자리좌회전45도-1(2회)', 2),
        ('LINE_LOST_TURN_RIGHT', '찐제자리우회전45도-1(5회)', 5),
    ]:
        motion_id = MotionCommandBridgeNode.ACTION_TO_MOTION_ID[action]
        assert aliases[motion_id] == name
        assert motions[name]['repeat_count'] == count


@pytest.mark.parametrize('side,digest', [
    ('좌', '839d8a59316168882e987e8610386a31df77005f5de0cb15ecd6c8b6f2713174'),
    ('우', '4d5a9601ab7d363b9e92d73e24283060aeba763a7075e692c94fd96e93123384'),
])
def test_recovery_cycle_matches_supplied_catalog(side, digest):
    motions = {m['name']: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']}
    fields = ('angles', 'torques', 'start_ms', 'time_ms')
    frames = [{k: f[k] for k in fields} for f in motions[f'찐라인복귀{side}회전45도(4회)']['frames']]
    encoded = json.dumps(frames, sort_keys=True, separators=(',', ':'))
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


@pytest.mark.parametrize("direction,count", [
    *[("LEFT", count) for count in range(1, 7)],
    *[("RIGHT", count) for count in (2, 3, 5, 7, 9)],
])
def test_ball_floor_turns_reach_registered_motion_with_matching_repeat_count(
    direction, count,
):
    from mission_control.motion_command_gate import normalize_general_action
    from mission_control.motion_decision_node import MotionDecisionNode

    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    catalog = json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    motions = {motion["name"]: motion for motion in catalog}
    approach = f"BALL_APPROACH_TURN_{direction}_{count}"
    initial = f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{count}"
    backward = f"BALL_PICKUP_POST_BACKWARD_TURN_{direction}_{count}"
    assert normalize_general_action(approach) == approach
    assert initial in MotionDecisionNode.PICKUP_INITIAL_ALIGN_ACTIONS
    assert backward in MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    assert initial in MotionCommandBridgeNode.PICKUP_INITIAL_ALIGN_ACTIONS
    assert backward in MotionCommandBridgeNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    for motion_id in (
        MotionCommandBridgeNode.ACTION_TO_MOTION_ID[approach],
        MotionCommandBridgeNode.ACTION_TO_MOTION_ID[
            f"POST_BALL_LINE_TURN_{direction}_{count}"
        ],
        MotionCommandBridgeNode.ACTION_TO_MOTION_ID[
            f"GOAL_CAMERA90_TURN_{direction}_{count}"
        ],
        MotionCommandBridgeNode.ACTION_TO_MOTION_ID[
            f"POST_SHOT_LINE_TURN_{direction}_{count}"
        ],
        MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS[initial],
    ):
        assert motions[aliases[motion_id]]["repeat_count"] == count


@pytest.mark.parametrize("count,angle", [(2, 15), (3, 30), (5, 45), (7, 65), (9, 95)])
def test_calibrated_right_turn_aliases_resolve_to_supplied_catalog(count, angle):
    from mission_control.motion_decision_planner import MotionDecisionPlanner

    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        motion["name"]: motion
        for motion in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    assert MotionDecisionPlanner.RIGHT_TURN_ANGLES_DEG[count] == angle
    for prefix, camera in (
        ("line_turn_right", 45), ("post_ball_line_turn_right", 45),
        ("pickup_camera_down_turn_right", 0), ("goal_camera_90_turn_right", 90),
    ):
        name = ("찐제자리우회전45도(9회)" if camera == 45 and count == 9
                else f"찐제자리우회전{camera}도-1({count}회)")
        assert aliases[f"{prefix}_{count}"] == name
        assert motions[name]["repeat_count"] == count


@pytest.mark.parametrize("motion_id,name,digest", [
    ("pickup_retreat_2", "찐후진실전(1회)",
     "17081fe0b604bb1ddb60c3ccded49a7840c8d24908ec59e178bcdc48257a3815"),
    ("pickup_first_backward_turn_right", "찐후진하고 제자리우회전(공)",
     "0568d0a09ea18182611e4ed34efe2d96943c0faf52107ce6c3683ab34f1c6005"),
    ("post_shot_default_turn_right", "찐기본자세에서 제자리 우회전(골대)",
     "d1adb58605677ed92b853640e7cb9125cb5c605f3eefab010c1397c78e84a844"),
    ("post_shot_default_turn_left", "찐기본자세에서 제자리좌회전(골대)",
     "455de963e21cd376d3da174e3c9da2423ec1964747dbf1beb7e1dab826a55009"),
    ("pickup_second_backward_turn_left", "찐후진에서 제자리좌회전(공)",
     "9735e143053bbc33496d68f0d3fe521432478df41bd86bcc03472c3577480418"),
    ("pickup_crab_left_0", "찐미세왼옆꽃게0도-1",
     "931415b970ab5d0dcf2bbdc4052079801f953a096a9ba28e3a5013856711929e"),
    ("goal_camera_90_crab_left", "찐미세왼옆꽃게90도-1",
     "dba9db69d2bc5342abe6a0812ae672b234acbf5687053b3dd7f40eab2a7bb7d7"),
])
def test_imported_motions_preserve_source_except_tolerance_and_added_turns(
    motion_id, name, digest,
):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    assert aliases[motion_id] == name
    motion = motions[name]
    assert motion["completion"]["position_tolerance_deg"] == 5.0
    original_turn_counts = {
        "post_shot_default_turn_right": 7,
        "post_shot_default_turn_left": 4,
        "pickup_first_backward_turn_right": 7,
        "pickup_second_backward_turn_left": 4,
    }
    if motion_id in original_turn_counts:
        # Verify the original prefix independently of the added turn sets.
        motion = dict(motion)
        motion["frames"] = motion["frames"][
            :2 + 2 * original_turn_counts[motion_id]
        ]
        last_frame = motion["frames"][-1]
        motion["max_seq_ms"] = last_frame["start_ms"] + last_frame["time_ms"]
    encoded = json.dumps(
        motion, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    )
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


@pytest.mark.parametrize("motion_id,original_count,turn_count,turn_pose,end_pose,duration_ms", [
    ("post_shot_default_turn_right", 7, 9, "제우오들25", "오뒤415", 3531),
    ("post_shot_default_turn_left", 4, 6, "제좌왼들25", "오뒤412", 2750),
    ("pickup_first_backward_turn_right", 7, 11, "제우오들25", "오뒤415", 4144),
    ("pickup_second_backward_turn_left", 4, 7, "제좌왼들25", "오뒤412", 3089),
])
def test_composite_extended_turn_sets_preserve_pose_and_timing(
    motion_id, original_count, turn_count, turn_pose, end_pose, duration_ms,
):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    motion = motions[aliases[motion_id]]
    frames = motion["frames"]
    assert motion["repeat_count"] == 1
    assert motion["playback_speed"] == 1.0
    assert motion["end_pose"] == end_pose
    assert [frame["name"] for frame in frames] == (
        ["왼들401", end_pose] + [turn_pose, end_pose] * turn_count
    )
    assert motion["max_seq_ms"] == duration_ms
    assert frames[-1]["start_ms"] + frames[-1]["time_ms"] == duration_ms
    for previous, current in zip(frames, frames[1:]):
        assert current["start_ms"] >= previous["start_ms"] + previous["time_ms"]

    # The appended sets must retain every field except their absolute start time.
    original = frames[:2 + 2 * original_count]
    period = original[-2]["start_ms"] - original[-4]["start_ms"]
    for cycle in range(1, turn_count - original_count + 1):
        for index, template in enumerate(original[-2:]):
            expected = dict(template)
            expected["start_ms"] += cycle * period
            assert frames[len(original) + (cycle - 1) * 2 + index] == expected


@pytest.mark.parametrize("completed,exit_name", [
    (0, "찐후진하고 제자리우회전(공)"),
    (1, "찐후진에서 제자리좌회전(공)"),
])
def test_pickup_retreat_exit_and_dwell_order(completed, exit_name):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    sequence = MotionCommandBridgeNode._pickup_motion_sequence({
        "source_command": {}, "mission_progress": {"pickups_completed": completed},
    })
    assert [aliases[motion_id] for motion_id in sequence[:-1]] == [
        "찐공잡기전후진-2(2회)", "찐공잡기리그랩까지 실전", "찐공확인자세",
        "찐후진실전(1회)", exit_name,
    ]
    assert sequence[-1] == MotionCommandBridgeNode.DWELL_MARKER
    assert MotionCommandBridgeNode.PICKUP_DWELL_SEC == 1.0
    assert MotionCommandBridgeNode.PICKUP_TURN_DWELL_SEC == 1.0


def test_general_right_uses_line_return_four_repeats_and_first_pickup_uses_composite():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    assert aliases[MotionCommandBridgeNode.ACTION_TO_MOTION_ID["RIGHT"]] == (
        "찐라인복귀우회전45 도(4회)"
    )
    sequence = MotionCommandBridgeNode._pickup_motion_sequence({
        "source_command": {}, "mission_progress": {"pickups_completed": 0},
    })
    assert sequence[-2] == "pickup_first_backward_turn_right"
    assert aliases[sequence[-2]] == "찐후진하고 제자리우회전(공)"


@pytest.mark.parametrize("count", [1, 4, 6, 8])
def test_unavailable_stationary_right_counts_are_not_accepted(count):
    from mission_control.motion_command_gate import normalize_general_action
    from mission_control.motion_decision_node import MotionDecisionNode

    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    approach = f"BALL_APPROACH_TURN_RIGHT_{count}"
    initial = f"BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_{count}"
    backward = f"BALL_PICKUP_POST_BACKWARD_TURN_RIGHT_{count}"
    assert normalize_general_action(approach) is None
    assert MotionCommandBridgeNode.motion_id_for_action(approach) is None
    assert initial not in MotionCommandBridgeNode.PICKUP_INITIAL_ALIGN_ACTIONS
    assert initial not in MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS
    assert initial not in MotionDecisionNode.PICKUP_INITIAL_ALIGN_ACTIONS
    assert backward not in MotionCommandBridgeNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    assert backward not in MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    assert f"post_ball_line_turn_right_{count}" not in aliases
    assert f"pickup_camera_down_turn_right_{count}" not in aliases


def test_pickup_crab_preparation_preserves_body_motion_with_camera_down():
    import copy

    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    expected = copy.deepcopy(motions["찐오뒤에서 기본자세(카메라45도)"])
    expected["name"] = "오뒤에서 기본자세(픽업 카메라0도)"
    camera_down = motions["전진0도-1(4회)"]["frames"][-1]["angles"]["0"]
    assert camera_down == -60.0
    for frame in expected["frames"]:
        frame["angles"]["0"] = camera_down
    motion = motions[aliases[MotionCommandBridgeNode.PICKUP_CRAB_PREPARE_MOTION_ID]]
    assert motion == expected
    assert motion["completion"]["position_tolerance_deg"] == 5.0


def test_goal_single_retreat_preserves_original_frames_and_three_cycle_pickup_motion():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())['motion_aliases']
    data = json.loads(RUNTIME_CATALOG_PATH.read_text())
    motions = {motion['name']: motion for motion in data['motions']}
    original = motions['후진실전-2(3회)']
    retreat = motions[aliases['goal_camera_90_backward_1']]
    assert original['repeat_count'] == 3
    assert retreat['repeat_count'] == 1
    assert {key: value for key, value in retreat.items() if key not in {'name', 'repeat_count'}} == {
        key: value for key, value in original.items() if key not in {'name', 'repeat_count'}
    }
    assert retreat['completion']['position_tolerance_deg'] == 5.0


def test_close_ball_backward_is_one_goal_retreat_cycle_with_camera_down():
    import copy

    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    expected = copy.deepcopy(motions[aliases["goal_camera_90_backward_1"]])
    expected["name"] = "후진실전-2(1회, 픽업 카메라0도)"
    for frame in expected["frames"]:
        frame["angles"]["0"] = -60.0
    recovery = motions[aliases["pickup_lost_ball_backward_1"]]
    assert recovery == expected
    assert recovery["repeat_count"] == 1
    pre_grasp = motions[aliases["pickup_pre_backward_camera_down"]]
    assert pre_grasp["name"] == "찐공잡기전후진-2(2회)"
    assert pre_grasp["repeat_count"] == 2
    assert motions[aliases["pickup_retreat_2"]]["repeat_count"] == 1
    assert "pickup_lost_ball_backward_1" not in MotionCommandBridgeNode.PICKUP_MOTION_TAIL
    assert "pickup_lost_ball_backward_1" not in MotionCommandBridgeNode.PICKUP_NO_BALL_MOTION_TAIL


@pytest.mark.parametrize('depth,expected_action,name,repeats', [
    (1.281, 'GOAL_CAMERA_90_FORWARD', '찐전진90(6회)', 3),
    (1.280, 'GOAL_CAMERA_90_FORWARD_2', '찐전진90(4회)', 2),
    (0.851, 'GOAL_CAMERA_90_FORWARD_2', '찐전진90(4회)', 2),
    (0.850, 'GOAL_CAMERA90_FINE_FORWARD_3', '찐미세90도-4(3회)', 3),
    (0.501, 'GOAL_CAMERA90_FINE_FORWARD_3', '찐미세90도-4(3회)', 3),
    (0.500, 'GOAL_CAMERA90_FINE_FORWARD_1', '찐미세90도-4(1회)', 1),
    (0.431, 'GOAL_CAMERA90_FINE_FORWARD_1', '찐미세90도-4(1회)', 1),
    (0.389, 'GOAL_CAMERA90_BACKWARD_1', '찐후진실전(1회)', 1),
])
def test_goal_depth_selection_resolves_to_requested_runtime_motion(
    depth, expected_action, name, repeats,
):
    from step.goal_navigation_planner import GoalNavigationPlanner

    command = GoalNavigationPlanner().plan({
        'detected': True, 'confidence': 0.9, 'depth_valid': True,
        'depth_m': depth, 'offset_x_px': 0, 'score_now': True,
    })
    assert command.action == expected_action
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        motion['name']: motion
        for motion in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']
    }
    motion_id = MotionCommandBridgeNode.motion_id_for_action(command.action)
    assert aliases[motion_id] == name
    # Experimental forward frames contain two gait cycles per repetition.
    assert motions[name]['repeat_count'] == repeats
    assert motions[name]['frames']


@pytest.mark.parametrize("action", [
    "RIGHT",
    "RECOVER_RIGHT_TURN_RIGHT_4",
    "RECOVER_LEFT_TURN_RIGHT_4",
    "BALL_APPROACH_RECOVER_RIGHT_4",
])
def test_line_return_right_four_preserves_catalog32_motion(action):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        motion["name"]: motion
        for motion in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    source_path = RUNTIME_CATALOG_PATH.parent / "catalog32_right_recovery_source.json"
    expected = json.loads(source_path.read_text())["motions"][0]
    assert expected["name"] == "찐라인복귀우회전45 도(4회)"
    assert expected["repeat_count"] == 4
    expected["completion"]["position_tolerance_deg"] = 5.0
    motion_id = MotionCommandBridgeNode.motion_id_for_action(action)
    assert motion_id == "line_recovery_right_4"
    assert aliases[motion_id] == expected["name"]
    # Preserve all source data, including torque flags, timing and end pose.
    assert motions[aliases[motion_id]] == expected


@pytest.mark.parametrize("alias,source_name", [
    ("goal_forward_to_default_90", "찐오뒤에서기본자세(0도)"),
    ("goal_fine_to_default_90", "찐미세오뒤에서기본자세"),
])
def test_goal_crab_preparation_changes_only_camera_and_pose_labels(alias, source_name):
    import copy

    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    source = copy.deepcopy(motions[source_name])
    prepared = copy.deepcopy(motions[aliases[alias]])
    camera_90 = motions[aliases["post_ball_camera_90"]]["frames"][-1]["angles"]["0"]
    assert camera_90 == 1.0
    assert len(source["frames"]) == len(prepared["frames"])
    for original_frame, prepared_frame in zip(source["frames"], prepared["frames"]):
        assert prepared_frame["angles"]["0"] == camera_90
        prepared_frame["angles"]["0"] = original_frame["angles"]["0"]
        prepared_frame["name"] = original_frame["name"]
    for key in ("name", "start_pose", "end_pose"):
        prepared[key] = source[key]
    assert prepared == source
    assert aliases["goal_fine_to_default"] == "찐미세오뒤에서기본자세"


@pytest.mark.parametrize("alias,name,frame_count,digest", [
    ("pickup_crab_right_0", "찐미세오옆꽃게0-1(1회)", 4,
     "955855ee4ea1ba22b9f2d65746e30a845f97b177505fa4a2f678de9737b004e5"),
    ("goal_camera_90_crab_right", "찐미세오옆꽃게90-1(1회)", 4,
     "ef4fd376232c750ce52d6f9c374f44fd7c665873bc1d2d7bbc65b6732eb90a3b"),
    ("pickup_fine_to_crab_right_0", "찐미세오뒤에서오옆꽃게0도", 2,
     "5058c68644682c32c76d1d07fba4e566d9089a87cd669954e8aa38ba5055c6f0"),
    ("goal_fine_to_crab_right_90", "찐미세오뒤에서 오옆꽃게90도", 2,
     "ba38d07ef6ba1ea0a4398f5d6667e45ab97463875deacf6cfc7e6a11ffd152b1"),
    ("goal_forward_to_crab_right_90", "찐오뒤에서 오옆꽃게90도", 2,
     "0579a746404e812d0b2fcee308bb59d0ac8301068371e4c4977cf996382b5f7b"),
])
def test_catalog29_right_crab_motions_preserve_source_except_tolerance(
    alias, name, frame_count, digest,
):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    assert aliases[alias] == name
    matches = [motion for motion in motions if motion["name"] == name]
    assert len(matches) == 1
    motion = matches[0]
    assert motion["repeat_count"] == 1
    assert len(motion["frames"]) == frame_count
    assert motion["completion"]["position_tolerance_deg"] == 5.0
    encoded = json.dumps(motion, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


def test_runtime_startup_pose_has_unambiguous_canonical_joint_angles():
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    matches = [
        (motion["name"], frame["angles"])
        for motion in motions
        for frame in motion["frames"]
        if frame["name"] == "오뒤412"
    ]
    assert matches
    canonical = next(angles for name, angles in matches if name == "찐오뒤412")
    assert set(canonical) == {str(i) for i in range(23)}
    for motion_name, angles in matches:
        assert angles == canonical, f"ambiguous startup pose in {motion_name}"


@pytest.mark.parametrize("count", [2, 4])
def test_pickup_camera_down_forward_request_resolves_to_existing_motion(count):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    motion_id = (MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_MOTION_IDS["STRAIGHT_2"]
                 if count == 4 else "ball_camera_down_forward_2")
    assert aliases[motion_id] == f"찐전진45({count}회)"
    assert motions[aliases[motion_id]]["frames"]
    assert motions[aliases[motion_id]]["repeat_count"] == count // 2


@pytest.mark.parametrize("count", [2, 4])
def test_pickup_forward_preserves_original_camera45_frames(count):
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    base = motions["찐전진45(4회)"]
    target = motions[f"찐전진45({count}회)"]
    assert target["repeat_count"] == count // 2
    for key in base.keys() - {"name", "repeat_count"}:
        assert target[key] == base[key]
    assert all(frame["angles"]["0"] > -40 for frame in target["frames"])


@pytest.mark.parametrize("camera,digest", [
    (0, "e9bfd6d522b113c852991f998a9f0b06b860e25e256106b658bb95c3ac5d05bc"),
    (45, "b88c956306ed650997fe3355b2374e34fd089e4e03eab08fb28d67282b6a1a72"),
    (90, "431236af4a758dac798edd50204dc1d309ffe4c3e01337d9425661d8c665d24e"),
])
def test_stationary_left_turns_preserve_latest_catalog29_data(camera, digest):
    """Preserve supplied left turns; camera 0 uses the 2026-09-29 update."""
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    catalog = {
        motion["name"]: motion
        for motion in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    prefix = {
        0: "pickup_camera_down_turn_left",
        45: "post_ball_line_turn_left",
        90: "goal_camera_90_turn_left",
    }[camera]
    motions = []
    for count in range(1, 7):
        name = f"찐제자리좌회전{camera}도-1({count}회)"
        assert aliases[f"{prefix}_{count}"] == name
        motion = catalog[name]
        assert motion["repeat_count"] == count
        assert motion["completion"]["position_tolerance_deg"] == 5.0
        motions.append(motion)
    encoded = json.dumps(
        motions, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    )
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


def test_hurdle_preserves_catalog31_source_except_five_degree_tolerance():
    source = json.loads((RUNTIME_CATALOG_PATH.parent / "catalog31_hurdle_source.json").read_text())
    catalog = json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    assert aliases["hurdle"] == "찐허들"
    matches = [m for m in catalog if m["name"] == aliases["hurdle"]]
    expected = copy.deepcopy(source)
    expected["completion"]["position_tolerance_deg"] = 5.0
    assert matches == [expected]
    assert all(m["name"] != "찐허들실전" for m in catalog)
    assert len(source["frames"]) == 15
    assert source["playback_speed"] == 0.95
    assert source["repeat_count"] == 1
    assert source["completion"]["position_tolerance_deg"] == 2.0
