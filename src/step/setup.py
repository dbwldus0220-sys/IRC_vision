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
        ],
    },
)
