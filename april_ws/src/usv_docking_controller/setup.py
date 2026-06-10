from setuptools import find_packages, setup

package_name = 'usv_docking_controller'

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
    maintainer='humiii',
    maintainer_email='hrzannat710@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "docking_controller = usv_docking_controller.docking_controller:main",
            "cmdvel_to_bluerov = usv_docking_controller.cmdvel_to_bluerov:main",
        ],
    },
)
