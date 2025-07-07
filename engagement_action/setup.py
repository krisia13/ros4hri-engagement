from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'engagement_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cristina',
    maintainer_email='cabadm00@estudiantes.unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'engagement_action = engagement_action.engagement_action_node:main',
            'led_control = engagement_action.led_control_node:main',
        ],
    },
)
