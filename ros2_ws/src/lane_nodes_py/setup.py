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
        ('share/' + package_name + '/config', ['config/objects.json']),
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
            'keeping_node = lane_nodes_py.keeping_node:main',
            'transform_node = lane_nodes_py.transform_node:main',
            'detection_node = lane_nodes_py.detection_node:main',
            'camera_controller_node = lane_nodes_py.camera_controller_node:main',
            'evaluation_keeping_node = lane_nodes_py.evaluation_keeping_node:main'
        ],
    },
)
