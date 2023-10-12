from setuptools import find_packages, setup

package_name = 'lane_nodes_py'

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
    maintainer='robert',
    maintainer_email='robsim378@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keeping = lane_nodes_py.keeping:main',
            'detection = lane_nodes_py.detection:main',
            'camera_controller = lane_nodes_py.camera_controller:main'
        ],
    },
)
