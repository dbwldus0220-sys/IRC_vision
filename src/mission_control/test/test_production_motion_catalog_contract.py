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
            "sdk_forward_3": "전진 실전(3회)",
            "sdk_turn_right_18": "우회전 실전(18회)",
            "sdk_return_default": "오뒤에서 기본자세로",
            "sdk_turn_in_place_right_6": "제자리우회전(6회)",
            "sdk_turn_in_place_left_6": "제자리좌회전(6회)",
            "sdk_turn_in_place_right_1": "제자리우회전(1회)",
            "sdk_turn_in_place_left_1": "제자리좌회전(1회)",
            "sdk_default_to_right_back": "기본자세에서 오뒤로",
            "sdk_default_to_left_back": "기본자세에서 윈뒤로",
            "sdk_turn_right_3": "우회전실전(3회)",
            "pickup": "공잡기리그랩까지 실전",
            "hurdle": "허들넘기 실전",
            "forward": "전진 실전(3회)",
        }
    }


def test_every_production_bridge_motion_id_has_an_approved_alias():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    assert set(MotionCommandBridgeNode.ACTION_TO_MOTION_ID.values()) <= set(
        aliases
    )
