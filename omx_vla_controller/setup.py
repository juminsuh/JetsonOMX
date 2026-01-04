from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'omx_vla_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='VLA (Vision-Language-Action) controller for OMX robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_dummy = omx_vla_controller.vla_dummy:main',
            'vla_bridge = omx_vla_controller.vla_bridge:main',
            'simple_controller = omx_vla_controller.simple_controller:main',
        ],
    },
)

