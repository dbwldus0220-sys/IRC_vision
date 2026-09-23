"""Lock the production bridge-to-C++ alias contract to approved entries."""

from pathlib import Path
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


@pytest.mark.parametrize("angle,count,digest", [
    (45, 4, "c4fbd3132aece9f8e6963bf4fe84c7fc6967ffcb83b313c689fc6484d5d619f0"),
    (45, 6, "bb878e9246457a9cac95cab98e355a94cdb190c095b7792bc2ea14710eaaadb9"),
    (45, 8, "de8f9c1980a0e5017b23844fc32c3afaa72d170fb5ffba28eea0b8f3f8251638"),
    (90, 4, "d4c189e862e4bc7bd5cb8da20881d5b1acfb03a178548562da2e4d7dcc5948e4"),
    (90, 6, "da4b6416af5dd3a7683f16a59ba9b045c229126be900ed48e434e970b20cba52"),
])
def test_experimental_forward_preserves_supplied_motion(angle, count, digest):
    aliases = yaml.safe_load(ALIAS_PATH.read_text(encoding="utf-8"))[
        "motion_aliases"
    ]
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text(encoding="utf-8"))[
        "motions"
    ]
    by_name = {motion["name"]: motion for motion in motions}
    expected_name = f"전진실험{angle}도({count}회)"
    expected_aliases = {
        (45, 4): ("sdk_forward_4", "line_forward_4", "post_ball_forward_4"),
        (45, 6): ("line_forward_6", "post_ball_forward_6"),
        (45, 8): ("post_ball_forward_8",),
        (90, 4): ("goal_camera_90_forward_4",),
        (90, 6): ("goal_camera_90_forward_6",),
    }
    for alias in expected_aliases[angle, count]:
        assert aliases[alias] == expected_name
    motion = by_name[expected_name]
    # robot_motions(21).json repeats an eight-frame, two-cycle gait.
    assert motion["repeat_count"] == count // 2
    assert len(motion["frames"]) == 8
    assert motion["playback_speed"] == pytest.approx(1.15)
    assert motion["completion"]["position_tolerance_deg"] == 5.0
    # Preserve the entire supplied motion, except the completion tolerance.
    encoded = json.dumps(
        motion, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    )
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


def test_post_ball_transition_contains_only_camera_and_stationary_pause():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]}
    assert MotionCommandBridgeNode.POST_BALL_GOAL_TRANSITION_SEQUENCE == (
        "post_ball_camera_90", MotionCommandBridgeNode.POST_BALL_CAMERA_DWELL_MARKER,
    )
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID["POST_BALL_GOAL_TRANSITION"] == "post_ball_camera_90"
    assert motions[aliases["post_ball_camera_90"]]["frames"]
    assert aliases["line_forward_6"] == "전진실험45도(6회)"


def test_goal_shot_matches_supplied_motion_with_five_degree_tolerance():
    motions = json.loads(RUNTIME_CATALOG_PATH.read_text(encoding="utf-8"))["motions"]
    target = next(motion for motion in motions if motion["name"] == "골넣기실전")
    encoded = json.dumps(target, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    assert hashlib.sha256(encoded.encode()).hexdigest() == (
        "44da2b881d854cd0a421ddbef40b755078bea116354203ffd1c6e329fc7683b9"
    )


def test_post_shot_fixed_turns_and_forward_resolve_to_runtime_motions():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    expected = {
        "POST_SHOT_TURN_RIGHT_9": ("기본자세에서 제자리 우회전(골대)", 1),
        "POST_SHOT_TURN_LEFT_4": ("기본자세에서 제자리좌회전(골대)", 1),
        "POST_SHOT_FORWARD": ("전진실험45도(6회)", 3),
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
            name = ("제자리우회전45도(9회)" if angle == 45 and count == 9
                    else f"제자리우회전{angle}도-1({count}회)")
            assert aliases[f"{alias_prefix}_{count}"] == name
            motion = by_name[name]
            assert motion["repeat_count"] == count
            assert len(motion["frames"]) == 2
            assert motion["playback_speed"] == 0.9
            assert motion["end_pose"] == (
                "오뒤415" if angle == 45 else f"오뒤415({angle}도)"
            )
            assert motion["frames"] == by_name[
                f"제자리우회전{angle}도-1(2회)"
            ]["frames"]

    assert aliases["stationary_turn_right"] == "제자리우회전45도(9회)"
    assert aliases["pickup_first_turn_right_7"] == "제자리우회전45도-1(7회)"
    assert aliases["pickup_first_turn_right_9"] == "제자리우회전45도(9회)"


def test_production_alias_catalog_contains_only_approved_aliases():
    payload = yaml.safe_load(ALIAS_PATH.read_text(encoding="utf-8"))
    assert payload == {
        "motion_aliases": {
            "sdk_pickup": "공잡기리그랩까지 실전",
            "sdk_hurdle": "허들실실전",
            "sdk_forward_4": "전진실험45도(4회)",
            "line_forward_2": "전진45도-1(2회)",
            "line_forward_4": "전진실험45도(4회)",
            "line_forward_6": "전진실험45도(6회)",
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
            "line_search_left_1": "제자리좌회전45도-1(1회)",
            "line_search_right_3": "제자리우회전45도-1(3회)",
            "stationary_turn_left": "제자리좌회전45도-1(6회)",
            "stationary_turn_right": "제자리우회전45도(9회)",
            "ball_camera_down_forward_2": "전진0도-1(2회)",
            "ball_camera_down_forward_4": "전진0도-1(4회)",
            "ball_camera_down_forward_6": (
                "전진0도-1(6회)"
            ),
            "ball_general_fine_forward_8": "미세45도-4",
            "pickup_fine_forward_0": "미세0도-4",
            "pickup_fine_prepare": "오뒤에서미세오뒤",
            "pickup_crab_prepare": "오뒤에서 기본자세(픽업 카메라0도)",
            "pickup_crab_right_0": "미세오옆꽃게0도",
            "pickup_crab_left_0": "미세왼옆꽃게0도-1",
            "pickup_pre_backward_camera_down": (
                "공잡기전후진-2(2회)"
            ),
            "pickup_grasp_check_pose": "공확인자세",
            "pickup_retreat_2": "후진실전-2(3회)",
            "pickup_second_backward_turn_left": "후진에서 제자리좌회전(공)",
            "pickup_first_backward_turn_right": "후진하고 제자리우회전(공)",
            "pickup_first_turn_right_7": "제자리우회전45도-1(7회)",
            "pickup_second_turn_left_3": "제자리좌회전45도-1(3회)",
            "pickup_first_turn_right_9": "제자리우회전45도(9회)",
            "pickup_first_to_right_back_camera_45": (
                "왼뒤에서 오뒤로(카메라45도)"
            ),
            **{
                f"pickup_camera_down_turn_left_{count}": (
                    f"제자리좌회전0도-1({count}회)"
                )
                for count in range(1, 7)
            },
            **{
                f"pickup_camera_down_turn_right_{count}": (
                    f"제자리우회전0도-1({count}회)"
                )
                for count in range(1, 10)
            },
            "post_ball_forward_4": "전진실험45도(4회)",
            "post_ball_forward_6": "전진실험45도(6회)",
            "post_shot_default_turn_right": "기본자세에서 제자리 우회전(골대)",
            "post_shot_default_turn_left": "기본자세에서 제자리좌회전(골대)",
            "post_shot_turn_right_9": "제자리우회전45도(9회)",
            "post_ball_forward_8": "전진실험45도(8회)",
            "post_ball_camera_90": "오뒤카메라90도",
            "goal_camera_90_forward_2": "전진90도-1(2회)",
            "goal_camera_90_forward_4": "전진실험90도(4회)",
            "goal_camera_90_forward_6": "전진실험90도(6회)",
            **{
                f"goal_camera_90_fine_forward_{count}": f"미세90도-4({count}회)"
                for count in range(1, 5)
            },
            "goal_camera_90_crab_right": "미세오옆꽃게90도-1",
            "goal_camera_90_crab_left": "미세왼옆꽃게90도-1",
            **{
                f"post_ball_line_turn_right_{count}": (
                    ("제자리우회전45도(9회)" if count == 9
                     else f"제자리우회전45도-1({count}회)")
                )
                for count in range(1, 10)
            },
            "post_ball_line_turn_left_1": "제자리좌회전45도-1(1회)",
            "post_ball_line_turn_left_2": "제자리좌회전45도-1(2회)",
            "post_ball_line_turn_left_3": "제자리좌회전45도-1(3회)",
            "post_ball_line_turn_left_4": "제자리좌회전45도-1(4회)",
            "post_ball_line_turn_left_5": "제자리좌회전45도-1(5회)",
            "post_ball_line_turn_left_6": "제자리좌회전45도-1(6회)",
            **{
                f"goal_camera_90_turn_right_{count}": (
                    f"제자리우회전90도-1({count}회)"
                )
                for count in (2, 3, 5, 7, 9)
            },
            **{
                f"goal_camera_90_turn_left_{count}": (
                    f"제자리좌회전90도-1({count}회)"
                )
                for count in range(1, 7)
            },
            "goal_shot": "골넣기실전",
            "goal_fine_to_default": "미세오뒤에서기본자세",
            "sdk_default_to_right_back": "기본에서 오뒤 실실전",
            "pickup": "공잡기리그랩까지 실전",
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
    assert "왼뒤에서기본자세(90도)" not in aliases.values()
    assert "왼뒤에서기본(0도)" not in aliases.values()
    assert aliases["goal_camera_90_turn_left_1"] == (
        "제자리좌회전90도-1(1회)"
    )
    assert aliases["goal_camera_90_turn_left_6"] == (
        "제자리좌회전90도-1(6회)"
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
        "stationary_turn_left": ("제자리좌회전45도-1(6회)", 6),
        "stationary_turn_right": ("제자리우회전45도(9회)", 9),
        "pickup_first_turn_right_7": ("제자리우회전45도-1(7회)", 7),
        "pickup_second_turn_left_3": ("제자리좌회전45도-1(3회)", 3),
        "pickup_first_turn_right_9": ("제자리우회전45도(9회)", 9),
        **{
            f"post_ball_line_turn_right_{count}": (
                (
                    "제자리우회전45도(9회)" if count == 9
                    else f"제자리우회전45도-1({count}회)"
                ),
                count,
            )
            for count in (2, 3, 5, 7, 9)
        },
        "post_ball_line_turn_left_1": ("제자리좌회전45도-1(1회)", 1),
        "post_ball_line_turn_left_2": ("제자리좌회전45도-1(2회)", 2),
        "post_ball_line_turn_left_3": ("제자리좌회전45도-1(3회)", 3),
        "post_ball_line_turn_left_4": ("제자리좌회전45도-1(4회)", 4),
        "post_ball_line_turn_left_6": ("제자리좌회전45도-1(6회)", 6),
        **{
            f"goal_camera_90_turn_right_{count}": (
                f"제자리우회전90도-1({count}회)",
                count,
            )
            for count in (2, 3, 5, 7, 9)
        },
        **{
            f"goal_camera_90_turn_left_{count}": (
                f"제자리좌회전90도-1({count}회)",
                count,
            )
            for count in range(1, 7)
        },
        **{
            f"pickup_camera_down_turn_left_{count}": (
                f"제자리좌회전0도-1({count}회)",
                count,
            )
            for count in range(1, 7)
        },
        **{
            f"pickup_camera_down_turn_right_{count}": (
                f"제자리우회전0도-1({count}회)",
                count,
            )
            for count in (2, 3, 5, 7, 9)
        },
    }

    for motion_id, (exact_name, repeat_count) in expected.items():
        assert aliases[motion_id] == exact_name
        assert motions_by_name[exact_name]["repeat_count"] == repeat_count

    assert motions_by_name["제자리좌회전90도-1(6회)"][
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


def test_production_motions_use_five_degree_final_tolerance():
    catalog = json.loads(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    motions_by_name = {motion["name"]: motion for motion in catalog}

    for motion in catalog:
        assert motion["completion"][
            "position_tolerance_deg"
        ] == 5.0

    production_tolerance = motions_by_name[
        "공잡기리그랩까지 실전"
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
        "공잡기리그랩까지 실전": (
            8000,
            1,
            1.05,
            "음",
            "기본자세4",
        ),
        "공확인자세": (7045, 1, 1.0, "공확인자세", "공확인자세"),
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
        ("ball_general_fine_forward_8", "미세45도-4"),
        ("pickup_fine_forward_0", "미세0도-4"),
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
    base = motions['미세90도-4(1회)']
    assert motion['repeat_count'] == count
    assert len(motion['frames']) == 4
    assert motion['frames'] == base['frames']
    assert motion['start_pose'] == '미세오들4112(90도)'
    assert motion['end_pose'] == '미세오뒤4111(90도)'
    assert motion['playback_speed'] == 1.0
    assert motion['max_seq_ms'] == 3505


@pytest.mark.parametrize('name,digest', [
    ('미세0도-4', '85fc667137b6b50476fa88a4998065b72f33ed60fd06415676685adb0b446dfb'),
    ('미세45도-4', '7eb93d40e4fea5441dc4b603443bd5cc46f626ee6d4742895912d44dc649e4a0'),
    ('미세90도-4(1회)', '05a8bf2428764b71f8bd4cf6b4663227facd5a34cd489de6e977186e86af704f'),
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
    assert aliases[prepare_id] == "오뒤에서미세오뒤"
    assert motions[aliases[prepare_id]]["repeat_count"] == 1
    assert motions[aliases[prepare_id]]["completion"]["position_tolerance_deg"] == 5.0
    assert aliases["pickup_fine_forward_0"] == "미세0도-4"


def test_pickup_retreat_keeps_command_id_with_three_repeats():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())['motions']}
    retreat = motions[aliases['pickup_retreat_2']]
    assert aliases['pickup_retreat_2'] == '후진실전-2(3회)'
    assert retreat['repeat_count'] == 3
    assert MotionCommandBridgeNode.PICKUP_MOTION_TAIL[-1] == 'pickup_retreat_2'
    assert MotionCommandBridgeNode.PICKUP_NO_BALL_MOTION_TAIL[-1] == 'pickup_retreat_2'


@pytest.mark.parametrize("motion_id,name,digest", [
    ("pickup", "공잡기리그랩까지 실전",
     "f1b1755c63be0c370c1e00189b487f8b39a602a80276b50f23500adb8d31baea"),
    ("pickup_pre_backward_camera_down", "공잡기전후진-2(2회)",
     "c90ae240232d8c9b26fba83106c2908d43c4174577b30a401e44a67b6f00eee1"),
    ("pickup_crab_right_0", "미세오옆꽃게0도",
     "2d965494d9a9427b2a3ca30276c7dd2728fac95ea28e625e1fb91c386d1d92e4"),
    ("goal_camera_90_crab_right", "미세오옆꽃게90도-1",
     "561c3ef72c27b2280b1bd1de171b636bda187f42089517405812027687bf3264"),
])
def test_import15_requested_motions_match_source_with_five_degree_tolerance(
    motion_id, name, digest,
):
    """Preserve robot_motions(15).json data with only the approved tolerance edit."""
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
        ('LINE_LOST_TURN_LEFT', '제자리좌회전45도-1(1회)', 1),
        ('LINE_LOST_TURN_RIGHT', '제자리우회전45도-1(3회)', 3),
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
    frames = [{k: f[k] for k in fields} for f in motions[f'라인복귀{side}회전(4회)']['frames']]
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
        name = ("제자리우회전45도(9회)" if camera == 45 and count == 9
                else f"제자리우회전{camera}도-1({count}회)")
        assert aliases[f"{prefix}_{count}"] == name
        assert motions[name]["repeat_count"] == count


@pytest.mark.parametrize("motion_id,name,digest", [
    ("pickup_retreat_2", "후진실전-2(3회)",
     "c1374afd25c990b097fd4fdb360bb0a026bebe3dba5a5db35827acfdca6af2f5"),
    ("pickup_first_backward_turn_right", "후진하고 제자리우회전(공)",
     "061d94ea5a29696e788a89e54a2d84b982da94a2d76c2b3d556b4389655b89ce"),
    ("post_shot_default_turn_right", "기본자세에서 제자리 우회전(골대)",
     "8b26bcdc4c5746441510db602875cd4690203b70061d98cb7e39ac625e515d36"),
    ("post_shot_default_turn_left", "기본자세에서 제자리좌회전(골대)",
     "0b20c99e28d3e94b9c21041f5910aaeaf2c52f46f5abb37c3db11b028a700cf8"),
    ("pickup_second_backward_turn_left", "후진에서 제자리좌회전(공)",
     "426abc1983f53ee1c84f75f2be1aa51d2b917d6ab356483184f57f4a355437c5"),
    ("pickup_crab_left_0", "미세왼옆꽃게0도-1",
     "a9dcd5c3fc5e49a5c9c5894354ac7e15380405a0f8de3f016934ee407fe528e0"),
    ("goal_camera_90_crab_left", "미세왼옆꽃게90도-1",
     "8e8b765d52204375102aeb998ac2e5ef8f2f2e07dd09949ecc7ad64566f162be"),
])
def test_imported_motions_preserve_source_except_tolerance(
    motion_id, name, digest,
):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    motions = {
        m["name"]: m for m in json.loads(RUNTIME_CATALOG_PATH.read_text())["motions"]
    }
    assert aliases[motion_id] == name
    motion = motions[name]
    assert motion["completion"]["position_tolerance_deg"] == 5.0
    encoded = json.dumps(
        motion, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    )
    assert hashlib.sha256(encoded.encode()).hexdigest() == digest


@pytest.mark.parametrize("completed,exit_name", [
    (0, "후진하고 제자리우회전(공)"),
    (1, "후진에서 제자리좌회전(공)"),
])
def test_pickup_retreat_exit_and_dwell_order(completed, exit_name):
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    sequence = MotionCommandBridgeNode._pickup_motion_sequence({
        "source_command": {}, "mission_progress": {"pickups_completed": completed},
    })
    assert [aliases[motion_id] for motion_id in sequence[:-1]] == [
        "공잡기전후진-2(2회)", "공잡기리그랩까지 실전", "공확인자세",
        "후진실전-2(3회)", exit_name,
    ]
    assert sequence[-1] == MotionCommandBridgeNode.DWELL_MARKER
    assert MotionCommandBridgeNode.PICKUP_DWELL_SEC == 3.0


def test_general_right_keeps_nine_repeats_and_first_pickup_uses_composite():
    aliases = yaml.safe_load(ALIAS_PATH.read_text())["motion_aliases"]
    assert aliases[MotionCommandBridgeNode.ACTION_TO_MOTION_ID["RIGHT"]] == (
        "제자리우회전45도(9회)"
    )
    sequence = MotionCommandBridgeNode._pickup_motion_sequence({
        "source_command": {}, "mission_progress": {"pickups_completed": 0},
    })
    assert sequence[-2] == "pickup_first_backward_turn_right"
    assert aliases[sequence[-2]] == "후진하고 제자리우회전(공)"


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
    expected = copy.deepcopy(motions["오뒤에서 기본자세(카메라45도)"])
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
