from setuptools import setup

package_name = 'gazeboexample'

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
    maintainer='vinicioslugli',
    maintainer_email='vinicioslugli@gmail.com',
    description='A example of implementation on Gazebo simulator in ROS2 using python and rclpy library',
    license='GPLv3',
    tests_require=[],
    entry_points={
        'console_scripts': [
			'gazeboexampleendpoint = gazeboexample:main',
        ],
    },
)
