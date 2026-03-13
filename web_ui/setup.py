from setuptools import setup
import os
from glob import glob

package_name = 'web_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', glob('templates/*')),
        ('share/' + package_name + '/static/css', glob('static/css/*')),
        ('share/' + package_name + '/static/js', glob('static/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Web UI for Seract robotic inspection system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_ui_bridge = web_ui.web_ui_bridge:main',
            'web_ui_server = web_ui.web_ui_server:main',
        ],
    },
)

