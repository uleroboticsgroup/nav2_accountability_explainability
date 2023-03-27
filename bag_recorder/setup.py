import os
from glob import glob
from setuptools import setup

package_name = 'bag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laura',
    maintainer_email='laura@todo.todo',
    description='Nav2 bag recorder',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_recorder_srv = bag_recorder.bag_recorder_srv:main',
            'bag_recorder_scan_client = bag_recorder.bag_recorder_scan_client:main',
            'bag_recorder_btstatus_client = bag_recorder.bag_recorder_btstatus_client:main',
            'behavior_explainer = bag_recorder.behavior_explainer:main',
        ],
    },
)
