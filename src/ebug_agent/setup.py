import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ebug_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #('share/' + package_name, []),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='markd',
    maintainer_email='mark.diedericks830@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'TransformConverter = ebug_agent.TransformConverter:main',
            'MovementController = ebug_principal.MovementController:main',
        ],
    },
)
