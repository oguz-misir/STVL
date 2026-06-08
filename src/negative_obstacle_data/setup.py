from setuptools import find_packages, setup

package_name = 'negative_obstacle_data'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/synthetic_depth_params.yaml']),
        ('lib/' + package_name, [
            'scripts/arduino_serial_driver',
            'scripts/gazebo_data_collector',
            'scripts/gazebo_label_generator',
            'scripts/synthetic_depth_generator',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oguz',
    maintainer_email='dr.oguz.misir@gmail.com',
    description='Synthetic depth dataset generation pipeline.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synthetic_depth_generator = negative_obstacle_data.synthetic_depth_generator:main',
            'synthetic_label_visualizer = negative_obstacle_data.synthetic_label_visualizer:main',
            'synthetic_split_builder = negative_obstacle_data.synthetic_split_builder:main',
            'synthetic_dataset_qc = negative_obstacle_data.synthetic_dataset_qc:main',
            'gazebo_data_collector = negative_obstacle_data.gazebo_data_collector:main',
            'gazebo_label_generator = negative_obstacle_data.gazebo_label_generator:main',
            'arduino_serial_driver = negative_obstacle_data.arduino_serial_driver:main',
            'motor_control_gui = negative_obstacle_data.motor_control_gui:main',
        ],
    },
)
