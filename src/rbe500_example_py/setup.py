from setuptools import find_packages, setup
import os

from glob import glob

package_name = 'rbe500_example_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'gazebo.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'demo.launch.py'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description','rviz', 'car.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', 'car_arena.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hylander',
    maintainer_email='stevenhyland1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_robot_control = rbe500_example_py.basic_robot_control:main',
            'picknplace = rbe500_example_py.picknplace:main',
            'picknplace_ik = rbe500_example_py.picknplace_ik:main',
            'spawn_manipulator = rbe500_example_py.spawn_manipulator:main',
        ],
    },
)
