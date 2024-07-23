from setuptools import find_packages, setup
import os

package_name = 'steeleagle_mission'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

cnc_protocol_dir = os.path.join('cnc_protocol')
cnc_protocol_files = package_files(cnc_protocol_dir)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name + '/implementation/cloudlets', [
        #     os.path.join('implementation', 'cloudlets', 'PartialOffloadCloudlet.py'),
        #     os.path.join('implementation', 'cloudlets', 'PureOffloadCloudlet.py'),
        # ]),
        # ('lib/' + package_name + '/implementation/drones', [
        #     os.path.join('implementation', 'drones', 'MavlinkDrone.py'),
        #     os.path.join('implementation', 'drones', 'ModalAISeekerDrone.py'),
        #     os.path.join('implementation', 'drones', 'ParrotAnafiDrone.py'),
        # ]),
        # ('lib/' + package_name + '/interfaces', [
        #     os.path.join(package_name, 'interfaces', 'CloudletItf.py'),
        #     os.path.join(package_name, 'interfaces', 'DroneItf.py'),
        #     os.path.join(package_name, 'interfaces', 'Task.py'),
        #     os.path.join(package_name, 'interfaces', 'Transition.py'),
        # ]),
        # ('lib/' + package_name + '/cnc_protocol', cnc_protocol_files),
        
    ],
    install_requires=[
        'setuptools',    
    ],
    zip_safe=True,
    maintainer='xianglic',
    maintainer_email='xianglic@andrew.cmu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hypervisor = steeleagle_mission.hypervisor:main',
            'supervisor = steeleagle_mission.supervisor:main',
            'control_plane = steeleagle_mission.control_plane_ros:main'
        ],
    },
)
