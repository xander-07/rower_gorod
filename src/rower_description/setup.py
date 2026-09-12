from setuptools import find_packages, setup

package_name = 'rower_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/rower.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/robot_geometry.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='80409749+xander-07@users.noreply.github.com',
    description='Robot geometry and TF description for the Waveshare UGV02 rower robot.',
    license='MIT',
)
