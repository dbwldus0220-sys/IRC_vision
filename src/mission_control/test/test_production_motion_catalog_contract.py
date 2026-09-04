"""Lock the production bridge-to-C++ alias contract to approved entries."""

from pathlib import Path

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
import yaml


ALIAS_PATH = (
    Path(__file__).resolve().parents[2]
    / "irc_step_motion_executor"
    / "config"
    / "motion_aliases.yaml"
)


def test_production_alias_catalog_contains_only_approved_aliases():
    payload = yaml.safe_load(ALIAS_PATH.read_text(encoding="utf-8"))
    assert payload == {
        "motion_aliases": {
            "sdk_pickup": "공잡기리그랩까지 실전",
            "sdk_hurdle": "허들넘기 실전",
            "sdk_forward_4": "전진실실전(4회)",
            "line_forward_2": "전진실실전(2회)",
            "line_forward_4": "전진실실전(4회)",
            "line_forward_6": "전진실실전(6회)",
            "line_forward_8": "전진실실전(8회)",
            "line_forward_10": "전진실실전(10회)",
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
            "line_recovery_left_4": "라인복귀좌회전(4회)",
            "line_recovery_left_5": "라인복귀좌회전(5회)",
            "line_recovery_left_6": "라인복귀좌회전(6회)",
            "line_recovery_left_7": "라인복귀좌회전(7회)",
            "line_recovery_left_8": "라인보귀좌회전(8번)",
            "line_recovery_right_4": "라인복귀우회전(4회)",
            "line_recovery_right_5": "라인복귀우회전(5회)",
            "line_recovery_right_6": "라인복귀우회전(6회)",
            "line_recovery_right_7": "라인복귀우회전(7회)",
            "line_recovery_right_8": "라인보귀우회전(8번)",
            "stationary_turn_left": "제자리좌회전실실전(6번)",
            "stationary_turn_right": "제자리우회전실실전(6번)",
            "sdk_turn_right_15": "우회전실실전(15회)",
            "pickup_pre_backward": "공 잡기 전 후진 실전",
            "sdk_return_default": "오뒤에서 기본자세",
            "sdk_default_to_right_back": "기본자세에서 오뒤로",
            "pickup": "공잡기리그랩까지 실전",
            "hurdle": "허들넘기 실전",
            "forward": "전진실실전(10회)",
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


def test_every_production_bridge_motion_id_has_an_approved_alias():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    assert set(MotionCommandBridgeNode.ACTION_TO_MOTION_ID.values()) <= set(
        aliases
    )
