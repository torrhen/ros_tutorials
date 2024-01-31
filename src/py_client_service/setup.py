from setuptools import find_packages, setup

package_name = 'py_client_service'

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
    maintainer='torrhen',
    maintainer_email='torrhen@todo.todo',
    description='Python client service tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'service = py_client_service.add_three_ints_service:main',
		'client = py_client_service.add_three_ints_client:main',
        ],
    },
)
