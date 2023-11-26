from setuptools import find_packages, setup
from glob import glob

package_name = 'my_avoid_sys'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_picture_gui = my_avoid_sys.depth_picture_gui:main',
            'obstacle_detect = my_avoid_sys.obstacle_detect:main',
            'teleop_twist = my_avoid_sys.teleop_twist:main',
            'robot_controller = my_avoid_sys.robot_controller:main',
            'data_transmission = my_avoid_sys.data_transmission:main',
        ],
    },
)
