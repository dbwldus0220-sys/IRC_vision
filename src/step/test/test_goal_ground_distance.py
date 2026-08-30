"""Tests for goal ground-plane distance reporting."""

import pytest

from step.goal_analyzer import GoalAnalyzer


def test_goal_projection_reports_ground_plane_distance():
    """Expose pitch-corrected robot-floor range separately from optical Z."""
    analyzer = object.__new__(GoalAnalyzer)
    analyzer.fx = 600.0
    analyzer.fy = 600.0
    analyzer.cx = 640.0
    analyzer.cy = 360.0
    analyzer.camera_pitch_down_deg = 45.0
    analyzer.camera_forward_offset_m = 0.0

    projection = analyzer._project(640, 360, 1.0)

    assert projection[3] == pytest.approx(1.0)
    assert projection[4] == pytest.approx(2 ** -0.5)
