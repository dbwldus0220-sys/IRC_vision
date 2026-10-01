"""Keep image and ground control separate and fit only the approaching segment."""
import pytest
from step.line_navigation_planner import LineNavigationPlanner, NavigationConfig, line_heading
from step.yolo_line_analyzer import analyze_ground_line, LinePoint


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
