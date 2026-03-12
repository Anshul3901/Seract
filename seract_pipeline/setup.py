from setuptools import find_packages, setup

package_name = 'seract_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pipeline.yaml']),
        ('share/' + package_name + '/launch', ['launch/mock_stack.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 pipeline package for seract',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pipeline=seract_pipeline.pipeline:main',
            'calib_helper=seract_pipeline.calib_helper:main',
            'usb_camera_node=seract_pipeline.usb_camera_node:main',
        ],
    },
)

