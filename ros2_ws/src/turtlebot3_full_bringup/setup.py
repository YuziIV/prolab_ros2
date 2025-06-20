from setuptools import setup

package_name = 'turtlebot3_full_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_bringup.launch.py']),
        ('share/' + package_name + '/maps', ['maps/playground_map.yaml']),
        ('share/' + package_name + '/maps', ['maps/playground_map_edited.pgm']),        
        ('share/' + package_name + '/worlds', ['worlds/playground.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='your@email.com',
    description='Full bringup for TurtleBot3 with Gazebo, Nav2 and RViz2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)