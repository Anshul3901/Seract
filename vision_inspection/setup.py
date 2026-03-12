from setuptools import find_packages, setup

package_name = 'vision_inspection'

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
    maintainer='gixadmin',
    maintainer_email='prazy391@gmail.com',
    description='Vision inspection pipeline for defect detection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_inspection_node = vision_inspection.vision_inspection_node:main',
        ],
    },
)
