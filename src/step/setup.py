from setuptools import find_packages, setup

package_name = 'step'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/best.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geonwoo',
    maintainer_email='geonwoo@todo.todo',
    description='ROS 2 mission vision for the IRC STEP humanoid robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo26_detector=step.yolo26_detector:main',
            'yolo_line_analyzer=step.yolo_line_analyzer:main',
            'ball_analyzer=step.ball_analyzer:main',
            'ball_navigation_controller=step.ball_navigation_controller:main',
            'goal_analyzer=step.goal_analyzer:main',
            'goal_navigation_controller=step.goal_navigation_controller:main',
            'hurdle_analyzer=step.hurdle_analyzer:main',
            'hurdle_navigation_controller=step.hurdle_navigation_controller:main',
            'unified_vision_node=step.unified_vision_node:main',
            'motion_decision_node=step.motion_decision_node:main',
            'rgbd_visual_odometry=step.rgbd_visual_odometry:main',
            'line_debug_monitor=step.line_debug_monitor:main',
            'line_path_visualizer=step.line_path_visualizer:main',
            'line_navigation_controller=step.line_navigation_controller:main',
            'imu_line_pose_estimator=step.imu_line_pose_estimator:main',
            'step_motion_pose_test=step.step_motion_pose_test:main',
            'mission_state_estimator=step.mission_state_estimator:main',
            'mission_map_visualizer=step.mission_map_visualizer:main',
        ],
    },
)
