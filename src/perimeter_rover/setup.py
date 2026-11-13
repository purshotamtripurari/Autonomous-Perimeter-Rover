from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'perimeter_rover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle'),
         glob('models/turtlebot3_waffle/*')),
        (os.path.join('share', package_name, 'models', 'standing_person'),
         glob('models/standing_person/*.*')),
        (os.path.join('share', package_name, 'models', 'standing_person', 'meshes'),
         glob('models/standing_person/meshes/*')),
        (os.path.join('share', package_name, 'models', 'standing_person', 'materials', 'textures'),
         glob('models/standing_person/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'standing_person', 'thumbnails'),
         glob('models/standing_person/thumbnails/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    package_data={'perimeter_rover': ['templates/*']},
    zip_safe=True,
    maintainer='Perimeter Rover Maintainer',
    maintainer_email='maintainer@example.com',
    description='TurtleBot3 Waffle Gazebo surveillance simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'surveillance_node = perimeter_rover.surveillance_node:main',
            'patrol_node = perimeter_rover.patrol_node:main',
            'dynamic_obstacle_node = perimeter_rover.dynamic_obstacle_node:main',
            'web_controller_node = perimeter_rover.web_controller_node:main',
        ],
    },
)
