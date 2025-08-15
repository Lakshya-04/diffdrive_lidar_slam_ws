from setuptools import find_packages, setup

package_name = 'mobile_robot_utils'

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
    maintainer='Lakshya-04',
    maintainer_email='100lakshyaagarwal@gmail.com',
    description='Utility nodes for mobile robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_twist_server = mobile_robot_utils.marker_twist_server:main',
            'frame_fix_node = mobile_robot_utils.frame_fix_node:main',
            'odom_to_path_node = mobile_robot_utils.odom_to_path:main',
        ],
    },
)
