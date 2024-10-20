from setuptools import find_packages, setup

package_name = 'tndi24'

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
    maintainer='dronkab',
    maintainer_email='perrusquia832@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "px4_driver = tndi24.px4_driver:main",
        "cross_window = tndi24.cross_window:main",
        "state_machine = tndi24.state_machine:main",
        "trajectory_follower = tndi24.trajectory_follower:main",
        "window_detector = tndi24.window_detector:main",
        "yolo_detector = tndi24.yolo_detector:main"
        ],
    },
)
