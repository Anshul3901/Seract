from setuptools import find_packages, setup

package_name = 'obj_bbox'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obj_bbox_node = obj_bbox.obj_bbox:main',
            'yolo_bbox_node = obj_bbox.yolo_bbox:main',
        ],
    },
)
