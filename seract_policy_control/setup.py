import sys
import os
from setuptools import setup

# Workaround for colcon build issue: ignore unknown command-line options
# Colcon passes various options that setuptools doesn't recognize
# Filter out all unknown -- options before setup() is called
unknown_colcon_options = [
    '--uninstall', '--editable', '--build-directory', '--build-base',
    '--dist-dir', '--egg-base', '--install-base', '--install-headers',
    '--install-lib', '--install-platlib', '--install-scripts', '--install-data',
    '--exec-prefix', '--install-purelib', '--install-layout'
]

# Remove unknown options (both with and without values)
filtered_argv = [sys.argv[0]]  # Keep script name
i = 1
while i < len(sys.argv):
    arg = sys.argv[i]
    should_keep = True
    
    # Check if this is an unknown option
    for opt in unknown_colcon_options:
        if arg == opt or arg.startswith(opt + '='):
            should_keep = False
            # If it's an option with value, skip the next arg too (if it's the value)
            if arg == opt and i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith('-'):
                i += 1  # Skip the value
            break
    
    # Also filter out absolute paths that look like build directories
    if should_keep and os.path.isabs(arg) and ('build' in arg.lower() or 'install' in arg.lower()):
        # Check if it's a build/install directory path
        if any(part in arg for part in ['/build/', '/install/', 'build/', 'install/']):
            should_keep = False
    
    if should_keep:
        filtered_argv.append(arg)
    i += 1

sys.argv = filtered_argv

package_name = 'seract_policy_control'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/seract_policy_control']),
        ('share/seract_policy_control', ['package.xml']),
        ('share/seract_policy_control/launch', [
            'launch/policy_control.launch.py',
            'launch/run_stack.launch.py',
            'launch/joint_jogger.launch.py',
            'launch/policy_bridge.launch.py',
            'launch/orch.launch.py',
        ]),
        ('share/seract_policy_control/config', ['config/policy_control.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Direct LeRobot controller publishing JointTrajectory',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lerobot_policy_controller = seract_policy_control.lerobot_policy_controller:main',
            'joint_jogger = seract_policy_control.joint_jogger:main',
            'policy_bridge = seract_policy_control.policy_bridge:main',
        ],
    },
)

