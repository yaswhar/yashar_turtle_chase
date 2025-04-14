from setuptools import find_packages, setup
from glob import glob

package_name = 'yashar_turtle_chase'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yashar Zafari',
    maintainer_email='zafari.h.yashar@gmail.com',
    description='Dynamic leader-follower turtlesim demo with leader switching service',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtles = yashar_turtle_chase.spawn_turtles:main',
            'dynamic_follower = yashar_turtle_chase.dynamic_follower:main',
        ],
    },
    data_files=[
        # Install package.xml into the share directory
        ('share/yashar_turtle_chase', ['package.xml']),
        # Install launch files (all .launch.py files in the launch folder)
        ('share/yashar_turtle_chase/launch', glob('launch/*.launch.py'))
    ],
)
