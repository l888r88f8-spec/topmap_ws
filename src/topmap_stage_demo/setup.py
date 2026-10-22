import os
from glob import glob

from setuptools import setup

package_name = 'topmap_stage_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
        (os.path.join('share', package_name, 'tools'),
         glob('tools/*.py') + glob('tools/*.cpp')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='TopMap stage visualization and interactive RViz planning demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stage_publisher = topmap_stage_demo.publisher:main',
            'planning_publisher = topmap_stage_demo.planning:main',
            'interactive_planner = topmap_stage_demo.interactive:main',
        ],
    },
)
