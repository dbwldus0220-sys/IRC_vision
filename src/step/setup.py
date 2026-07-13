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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geonwoo',
    maintainer_email='geonwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'look_ground=step.look_ground:main',
            'look_gground=step.look_gground:main',
            'find_direct=step.find_direct:main',
            'find_ddirect=step.find_ddirect:main',
            'yolo26_detector=step.yolo26_detector:main',
            'yolo_line_analyzer=step.yolo_line_analyzer:main',
            'line_debug_monitor=step.line_debug_monitor:main',
            'line_path_visualizer=step.line_path_visualizer:main',
            'imu_line_pose_estimator=step.imu_line_pose_estimator:main',
            'step_motion_pose_test=step.step_motion_pose_test:main',
            'mission_state_estimator=step.mission_state_estimator:main',
            'mission_map_visualizer=step.mission_map_visualizer:main',
        ],
    },
)
