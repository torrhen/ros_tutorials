from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_parameters'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')) # include all launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='torrhen',
    maintainer_email='torrhennn@proton.me',
    description='Python parameter tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_param_node = py_parameters.py_parameters_node:main'
        ],
    },
)
