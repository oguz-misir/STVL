from setuptools import find_packages, setup

package_name = 'negative_obstacle_stvl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/semantic_drop_publisher']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oguz',
    maintainer_email='dr.oguz.misir@gmail.com',
    description='STVL integration for negative obstacle costmap marking.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_drop_publisher = negative_obstacle_stvl.semantic_drop_publisher:main',
        ],
    },
)
