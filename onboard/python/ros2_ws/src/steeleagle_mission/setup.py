from setuptools import find_packages, setup

package_name = 'steeleagle_mission'

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
    maintainer='xianglic',
    maintainer_email='xianglic@andrew.cmu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hypervisor = steeleagle_mission.hypervisor:main',
            'supervisor = steeleagle_mission.supervisor:main'
        ],
    },
)
