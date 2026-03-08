import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'safety_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml', 'safety_controller/params.yaml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/safety_controller/launch',
            glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/safety_controller/launch',
            glob.glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='sebastianag2002@gmail.com',
    description='Safety controller that prevents the racecar from crashing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_controller = safety_controller.safety_controller:main',
            'safety_controller_tester = safety_controller.safety_controller_tester:main',
        ],
    },
)
