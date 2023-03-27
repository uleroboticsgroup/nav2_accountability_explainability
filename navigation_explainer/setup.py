import os
from glob import glob
from setuptools import setup

package_name = 'navigation_explainer'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'navigation_explainer_srv = navigation_explainer.navigation_explainer_srv:main',
             'navigation_explainer_client = navigation_explainer.navigation_explainer_client:main',
        ],
    },
)
