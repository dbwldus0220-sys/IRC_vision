"""Outgoing corner geometry must not reuse the incoming tangent or image yaw."""

import math

import pytest

from step.yolo_line_analyzer import LinePoint, analyze_corner_exit


PARAMETERS = {
    'ground_projection_enabled': True,
    'ground_homography': [.001, 0., -.64, 0., -.001, 1., 0., 0., 1.],
    'ground_fit_min_forward_m': .1,
    'ground_fit_max_forward_m': 1.,
}


def path(angle):
    direction_x, direction_z = math.sin(math.radians(angle)), math.cos(math.radians(angle))
    ground = [(0., .2), (0., .4)] + [
        (distance * direction_x, .4 + distance * direction_z) for distance in (.1, .2, .3)
    ]
    return [LinePoint(640 + right * 1000, 1000 - forward * 1000, .9) for right, forward in ground]


@pytest.mark.parametrize('angle', [-90., -70., -50., -30., 30., 50., 70., 90.])
def test_outgoing_heading_includes_sideways_segments(angle):
    result = analyze_corner_exit(path(angle), 1280, 720, PARAMETERS, corner_start_index=1)
    assert result['corner_exit_ground_valid']
    assert result['corner_exit_ground_heading_deg'] == pytest.approx(angle)


@pytest.mark.parametrize('index', [None, True, -1, 4, 99])
def test_missing_or_stale_corner_index_never_fits_outgoing_path(index):
    result = analyze_corner_exit(path(90), 1280, 720, PARAMETERS, corner_start_index=index)
    assert not result['corner_exit_ground_valid']
    assert result['corner_exit_image_heading_deg'] is None


@pytest.mark.parametrize('changes', [
    {'ground_projection_enabled': False}, {'ground_homography': [0.] * 9},
    {'ground_fit_min_forward_m': .8},
])
def test_image_direction_remains_separate_when_ground_unavailable(changes):
    result = analyze_corner_exit(path(-90), 1280, 720, {**PARAMETERS, **changes}, corner_start_index=1)
    assert not result['corner_exit_ground_valid']
    assert result['corner_exit_ground_heading_deg'] is None
    assert result['corner_exit_image_heading_deg'] == pytest.approx(-90.)


def test_nonstraight_exit_is_not_a_trusted_ground_heading():
    points = path(90)
    points[3] = LinePoint(points[3].x, points[3].y - 200, .9)
    result = analyze_corner_exit(points, 1280, 720, PARAMETERS, corner_start_index=1)
    assert not result['corner_exit_ground_valid']
