from setuptools import find_packages, setup

package_name = 'negative_obstacle_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oguz',
    maintainer_email='dr.oguz.misir@gmail.com',
    description='Geometric risk perception pipeline.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
