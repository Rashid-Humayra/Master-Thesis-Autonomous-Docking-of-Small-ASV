from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'apriltag_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/apriltag_tracker']),
    ('share/apriltag_tracker', ['package.xml']),
    (os.path.join('share', 'apriltag_tracker', 'calibration_simulation'),
        glob('calibration_simulation/*')),
    # (os.path.join('share', 'apriltag_tracker', 'calibration_oldold'),
    #     glob('calibration_oldold/*')),
    (os.path.join('share', 'apriltag_tracker', 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humiii',
    maintainer_email='humiii@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_tracker = apriltag_tracker.apriltag_node_three_tags_copy:main',
            'ekf_filter = apriltag_tracker.ekf_node:main',
            'los_planner = apriltag_tracker.los_planner:main',
            'cmd_thrust = apriltag_tracker.cmdvel_to_thrust:main',
        ],
    },
)
