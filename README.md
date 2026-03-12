# README: How to Execute the ROS 2 Packages

This guide assumes you already copied the ROS 2 packages into a workspace on your machine.

---

## 1. Open a terminal
Start in the root of your ROS 2 workspace.

Example:
```bash
cd ~/your_ros2_ws
```

Your workspace should look something like this:
```bash
~/your_ros2_ws/
├── src/
│   ├── package_1/
│   ├── package_2/
│   └── ...
├── build/
├── install/
└── log/
```

If you only copied the packages, make sure they are inside the `src/` folder.

---

## 2. Source ROS 2
Before building or running anything, source your ROS 2 installation.

Example for ROS 2 Humble:
```bash
source /opt/ros/humble/setup.bash
```

If you use a different ROS 2 distro, replace `humble` with your version.

---

## 3. Go to the workspace
```bash
cd ~/your_ros2_ws
```

---

## 4. Install dependencies
If the packages use external ROS dependencies, install them with `rosdep`.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` is not initialized yet, run:
```bash
sudo rosdep init
rosdep update
```

---

## 5. Build the workspace
Build all packages with `colcon`.

```bash
colcon build
```

If you want cleaner terminal output:
```bash
colcon build --symlink-install
```

If you only want to build one package:
```bash
colcon build --packages-select <package_name>
```

Example:
```bash
colcon build --packages-select my_robot_bringup
```

---

## 6. Source the workspace overlay
After building, source the workspace so ROS 2 can find your new packages.

```bash
source install/setup.bash
```

You need to do this in every new terminal before running the packages.

---

## 7. Verify the packages are visible
Check that ROS 2 can see the packages.

```bash
ros2 pkg list | grep <package_name>
```

Example:
```bash
ros2 pkg list | grep my_robot_bringup
```

If the package appears, the build and sourcing worked.

---

## 8. Run a package node
To run a node directly:

```bash
ros2 run <package_name> <executable_name>
```

Example:
```bash
ros2 run my_robot_pkg controller_node
```

---

## 9. Run a launch file
If the package uses launch files:

```bash
ros2 launch <package_name> <launch_file_name>
```

Example:
```bash
ros2 launch my_robot_bringup bringup.launch.py
```

---

## 10. Check active nodes and topics
After launching, verify everything is running.

List nodes:
```bash
ros2 node list
```

List topics:
```bash
ros2 topic list
```

Echo a topic:
```bash
ros2 topic echo /topic_name
```

---

## 11. Rebuild after making code changes
If you edit source code, rebuild:

```bash
cd ~/your_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

If you changed only one package:
```bash
colcon build --packages-select <package_name> --symlink-install
source install/setup.bash
```

---

## 12. Common troubleshooting

### Package not found
Make sure:
- the package is inside `src/`
- the workspace was built successfully
- `source install/setup.bash` was run

### Dependency errors
Run:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build fails
Clean and rebuild:
```bash
rm -rf build install log
colcon build --symlink-install
```

### Launch file not found
Check that the launch file exists in the package’s `launch/` directory and that the package was built.

### Executable not found
Make sure the executable is properly installed in the package configuration (`setup.py` for Python packages or `CMakeLists.txt` for C++ packages).

---

## 13. Recommended terminal workflow
Every time you open a new terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/your_ros2_ws
source install/setup.bash
```

Then run your node or launch file.

---

## 14. Example full workflow
```bash
source /opt/ros/humble/setup.bash
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch <package_name> <launch_file_name>
```

---

## 15. Replace these placeholders
Before using this README, replace:
- `~/your_ros2_ws` with your actual workspace path
- `<package_name>` with your actual package name
- `<executable_name>` with the node executable name
- `<launch_file_name>` with the launch file you want to run

---

## 16. Optional: add auto-source to `.bashrc`
To avoid sourcing ROS 2 manually every time, add this to your `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/your_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Only do this if this is the main workspace you want loaded by default.

