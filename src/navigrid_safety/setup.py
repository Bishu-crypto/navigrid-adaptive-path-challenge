import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'navigrid_safety'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NaviGrid Team',
    maintainer_email='competition@navigrid.org',
    description='High priority dynamic safety override system for NaviGrid Challenge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = navigrid_safety.safety_node:main',
            'twist_priority_mux = navigrid_safety.twist_priority_mux:main',
        ],
    },
)
