from setuptools import find_packages, setup

package_name = 'tallysman_ros2'

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
    maintainer='nirali',
    maintainer_email='nirali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tallysman_gps = tallysman_ros2.tallysman_gps:main",
            "tallysman_gps_gga = tallysman_ros2.tallysman_gps_gga:main",
            "tallysman_gps_rmc = tallysman_ros2.tallysman_gps_rmc:main",
            "tallysman_gps_visualizer = tallysman_ros2.tallysman_gps_visualizer:main",
        ],
    },
)
