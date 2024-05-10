import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ebug'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #('share/' + package_name, []),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'calibration'), glob(os.path.join('calibration', '*.yaml'))),
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
            'RobotController = ebug.RobotController:main',

            'TransformConverter = ebug.TransformConverter:main',
            'MovementController = ebug.MovementController:main',

            'BoidsService = ebug.BoidsService:main',
            'DiscoService = ebug.DiscoService:main',

            'PseudoMovementController = ebug.PseudoMovementController:main',

            'PyGameDisplay = ebug.PyGameDisplay:main'

        ],
    },
)
