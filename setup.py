from setuptools import find_packages, setup
import os
 
package_name = 'tallysman_ros2'
 
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
 
def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files
 
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scripts'],
    data_files=package_files(data_files, ['launch/', 'params/', 'scripts/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gtirth',
    maintainer_email='tirth@indrorobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'rosie_control = rosie_control.rosie-control:main'
            "tallysman_gps = scripts.tallysman_gps:main",
            # "tallysman_gps_gga = tallysman_ros2.tallysman_gps_gga:main",
            # "tallysman_gps_rmc = tallysman_ros2.tallysman_gps_rmc:main",
            "tallysman_gps_visualizer = scripts.tallysman_gps_visualizer:main"
        ],
    },
)
