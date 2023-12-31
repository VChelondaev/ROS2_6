from setuptools import find_packages, setup

package_name = 'robot_app'

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
    maintainer='solmir',
    maintainer_email='v.chelondaev@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "depth = robot_app.depth_barrier_detector:main",
            "lidar = robot_app.lidar_barrier_detector:main"
        ],
    },
)
