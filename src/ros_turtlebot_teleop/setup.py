from setuptools import find_packages, setup

package_name = 'ros_turtlebot_teleop'

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
    maintainer='Gustavo Widman',
    maintainer_email='gustavo.widman@sou.inteli.edu.br',
    description='Ponderada 2 - ROS - TurtleBot Teleop',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'main = ros_turtlebot_teleop.main:main',
			'emergency = ros_turtlebot_teleop.emergency:main'
        ],
    },
)
