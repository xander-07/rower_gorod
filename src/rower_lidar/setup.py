from setuptools import find_packages, setup

package_name = 'rower_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='80409749+xander-07@users.noreply.github.com',
    description='ROS 2 driver for LDROBOT STL-19P / LD19-family lidar.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_node = rower_lidar.lidar_node:main',
        ],
    },
)
