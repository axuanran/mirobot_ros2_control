from setuptools import find_packages, setup

package_name = 'my_mirobot_package'

setup(
    name='my_mirobot_package',
    version='0.0.1',
    packages=['my_mirobot_package'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_mirobot_package']),
        ('share/my_mirobot_package', ['package.xml']),
    ],
    install_requires=['setuptools', 'wlkata-mirobot-python', 'pyserial'],
    zip_safe=False,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS2 package for controlling Mirobot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mirobot_teleop = my_mirobot_package.mirobot_teleop:main',
            'mirobot_teleop_with_wlkatapythonp = my_mirobot_package.mirobot_teleop_with_wlkatapythonp:main',
            'data_recorder = my_mirobot_package.data_recorder:main',
            'camera_publisher = my_mirobot_package.camera_publisher:main'
        ],
    },
)
