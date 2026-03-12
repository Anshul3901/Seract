from setuptools import setup

package_name = 'so101_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anshul',
    maintainer_email='prazy391@gmail.com',
    description='Motion control logic for SO-101 arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_place = so101_motion.pick_place_node:main',
            'joint_monitor = so101_motion.joint_tracking_monitor:main',
            'decision_movement_action_server = so101_motion.decision_movement_action_server:main',
            'scan_motion = so101_motion.scanning_motion:main',
        ],
    },
)
