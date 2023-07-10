from setuptools import setup

package_name = 'rpi_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Riyavong',
    maintainer_email='danjan36@gmail.com',
    description='4 mecanum wheel RPI4 with wireless controller',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'rpi_joy = rpi_robot.rpi_joy:main',
        'rpi_motor = rpi_robot.rpi_motor:main',
        'rpi_cam = rpi_robot.rpi_cam:main',
        'controller_cam = rpi_robot.controller_cam:main'
        ],
    },
)
