from setuptools import find_packages, setup

package_name = 'rower_base_bridge'

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
    description='ROS 2 bridge for the Waveshare UGV02 lower controller.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_bridge = rower_base_bridge.base_bridge:main',
        ],
    },
)
