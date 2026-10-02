"""Keep image and ground control separate and fit only the approaching segment."""
import pytest
from step.line_navigation_planner import LineNavigationPlanner, NavigationConfig, line_heading
from step.yolo_line_analyzer import analyze_ground_line, two_point_ground_candidate, LinePoint


def sample(**updates):
    info = dict(detected=True, filtered_heading_error_deg=0.,
                filtered_lateral_offset_norm=0., heading_quality=.95,
                geometry_quality=.95, detection_quality=.95,
                turn_angle_deg=0., turn_consistency=1., ground_projection_valid=False)
    info.update(updates)
    return info


@pytest.mark.parametrize('side', [-1, 1])
def test_default_recovery_uses_ground_heading_when_image_direction_disagrees(side):
    command = LineNavigationPlanner().plan(sample(
        filtered_heading_error_deg=-side * 19.5,
        filtered_lateral_offset_norm=side * 1.099,
        ground_projection_valid=True,
        ground_heading_error_deg=side * 18.21,
    ), .1)
    direction = 'RIGHT' if side > 0 else 'LEFT'
    assert command.motion == f'RECOVER_{direction}_TURN_{direction}_4'
    assert command.heading_error_deg == pytest.approx(side * 18.21)


def test_default_ground_mode_stops_without_valid_ground_fit():
    command = LineNavigationPlanner().plan(sample(filtered_heading_error_deg=20.), .1)
    assert not command.valid
    assert command.reason == 'invalid_ground_line_geometry'


def test_explicit_image_mode_tracks_despite_failed_ground_fit():
    planner = LineNavigationPlanner(NavigationConfig(heading_source="image"))
    command = planner.plan(sample(filtered_heading_error_deg=1.9,
        filtered_lateral_offset_norm=-.194, turn_angle_deg=79.5), .1)
    assert command.valid and command.motion == 'STRAIGHT'
    assert command.steering_error_deg == pytest.approx(1.9 - 24*.194 + .15*79.5)
    assert command.heading_error_deg == 1.9


@pytest.mark.parametrize('side', [-1, 1])
def test_image_turn_preserves_three_confirmations_and_direction_mapping(side):
    planner = LineNavigationPlanner(NavigationConfig(heading_source="image"))
    info = sample(filtered_heading_error_deg=side*10., turn_angle_deg=side*60.)
    assert [planner.plan(info,.1).motion for _ in range(3)] == [
        'STRAIGHT','STRAIGHT','LEFT' if side<0 else 'RIGHT']


@pytest.mark.parametrize('mode', ['image','ground'])
def test_mode_never_falls_back_to_other_coordinate_system(mode):
    info = sample(filtered_heading_error_deg=-20., ground_projection_valid=True,
                  ground_heading_error_deg=20.)
    assert line_heading(info,mode) == (-20. if mode=='image' else 20.)
    if mode=='image': info['filtered_heading_error_deg']=None
    else: info['ground_projection_valid']=False
    assert not LineNavigationPlanner(NavigationConfig(heading_source=mode)).plan(info,.1).valid


def test_image_raw_heading_is_allowed_and_bad_configuration_is_rejected():
    assert line_heading({'heading_error_deg':4.},'image') == 4.
    with pytest.raises(ValueError): LineNavigationPlanner(NavigationConfig(heading_source='auto'))


def corner_pixels():
    # Recorded WAIT screenshot markers, approximately reconstructed at camera resolution.
    xy=[(1086,878),(1090,780),(1092,702),(1093,638),(1095,585),(1098,550),
        (1118,517),(1147,493),(1190,475),(1252,461),(1369,449),(1493,441),(1617,435),(1733,428)]
    return [LinePoint((x-545)*1280/1192,(y-219)*720/671,1.) for x,y in xy]


def test_recorded_corner_whole_fit_fails_but_approach_segment_succeeds():
    points=corner_pixels()
    assert not analyze_ground_line(points,1280,720)['ground_projection_valid']
    fit=analyze_ground_line(points,1280,720,corner_start_index=5)
    assert fit['ground_projection_valid'] and fit['ground_fit_segment']=='PRE_CORNER'
    assert fit['ground_fit_point_count']==6
    assert fit['ground_heading_error_deg']==pytest.approx(-8.60,abs=.02)


@pytest.mark.parametrize('index',[0,1,-1,100,True])
def test_too_short_or_invalid_near_segment_cannot_fall_back_to_whole_corner(index):
    fit=analyze_ground_line(corner_pixels(),1280,720,corner_start_index=index)
    assert not fit['ground_projection_valid']
    assert fit['ground_fit_reason'] == 'too_few_segment_points'


def test_recorded_october2_corner_has_only_two_pre_corner_points():
    # Approximate markers from the 80 s screen recording, camera viewport at (532, 115).
    xy = [(1375, 827), (1374, 746), (1398, 671), (1458, 614),
          (1560, 570), (1638, 552), (1732, 533), (1800, 522)]
    points = [LinePoint(x - 532, y - 115, 1.) for x, y in xy]
    fit = analyze_ground_line(points, 1280, 720, corner_start_index=1)
    assert not fit['ground_projection_valid']
    assert fit['ground_fit_input_point_count'] == 2
    assert fit['ground_fit_reason'] == 'too_few_segment_points'
    assert fit['ground_heading_error_deg'] is None
    candidate = two_point_ground_candidate(points, 1280, 720, {}, corner_start_index=1)
    assert candidate is not None
    assert candidate['span_m'] == pytest.approx(.04417, abs=.0001)
    assert candidate['heading_deg'] == pytest.approx(8.604, abs=.02)


@pytest.mark.parametrize('changes', [
    {'corner_start_index': 0}, {'corner_start_index': True}, {'corner_start_index': None},
    {'parameters': {'ground_projection_enabled': False}},
    {'parameters': {'ground_homography': [0.] * 9}},
])
def test_two_point_candidate_rejects_invalid_projection_or_segment(changes):
    args = dict(points=[LinePoint(843., 712., 1.), LinePoint(842., 631., 1.)],
                image_width=1280, image_height=720, parameters={}, corner_start_index=1)
    args.update(changes)
    assert two_point_ground_candidate(**args) is None


def test_two_point_candidate_rejects_short_baseline():
    assert two_point_ground_candidate(
        [LinePoint(843., 712., 1.), LinePoint(842., 710., 1.)],
        1280, 720, {}, corner_start_index=1) is None


def test_real_analyzer_selects_current_corner_prefix_and_image_planner_tracks(monkeypatch):
    import json
    import rclpy
    from std_msgs.msg import String
    from step.yolo_line_analyzer import YoloLineAnalyzer
    rclpy.init()
    node = YoloLineAnalyzer()
    try:
        published = []
        monkeypatch.setattr(node.publisher, 'publish', lambda msg: published.append(json.loads(msg.data)))
        message = String(data=json.dumps({
            'image_width':1280, 'image_height':720, 'stamp':{'sec':123,'nanosec':0},
            'detections':[{'class_name':'line','confidence':1.,'center':[p.x,p.y],
                           'bbox':[p.x-5,p.y-5,p.x+5,p.y+5]} for p in corner_pixels()]}))
        for _ in range(3): node._detections_callback(message)
        result = published[-1]
        assert result['detected'] and result['corner_start_index']==5
        assert result['ground_fit_segment']=='PRE_CORNER' and result['ground_projection_valid']
        assert result['ground_fit_point_count']==6
        assert result['filtered_heading_error_deg']==pytest.approx(1.866,abs=.01)
        command=LineNavigationPlanner(NavigationConfig(heading_source="image")).plan(result,.1)
        assert command.valid and command.motion=='STRAIGHT'
    finally:
        node.destroy_node()
        rclpy.shutdown()
