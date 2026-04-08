# **Scan2Grid-TB3**

**Nano ROS1 perception project that converts TurtleBot3 laser scans and odometry into a simple 2D occupancy map in Gazebo.**

---

## 1. Concept

Scan2Grid-TB3 is a compact robotics perception project built around a TurtleBot3 operating in a Gazebo simulation. The core idea is simple: use **LiDAR range data** and **robot odometry** to estimate where obstacles lie in the environment, then project those obstacle points onto a **2D occupancy grid map**.

The project is intentionally lightweight and educational. It focuses on the geometric and implementation-level foundations of robotic mapping rather than on full-scale probabilistic SLAM with loop closure, graph optimization, or scan matching.

At its heart, this project answers one question:

**Given where the robot is and what the laser scanner sees, how do we turn that into a map?**

The mapping node subscribes to:
- `/odom` for robot pose.
- `/scan` for LiDAR measurements.

It publishes:

`/map` as a `nav_msgs/OccupancyGrid`

This makes it a clean first-principles mapping project in ROS1.

## 2. Idea

The idea behind Scan2Grid-TB3 is to build a minimal but interpretable mapping system by chaining together three standard robotics ingredients:

1. **Pose estimation from odometry**.
2. **Obstacle observation from laser scans**.
3. **Coordinate transformation into a discrete grid map**.

Instead of using a full SLAM stack as a black box, this project exposes the mechanics directly:
- the robot pose is read from odometry.
- each laser beam is converted into a 2D point in the robot frame.
- that point is transformed into the world frame.
- the world-frame coordinate is discretized into grid indices.
- the corresponding occupancy cell is marked as occupied.

This makes the project useful as:
- an educational ROS mapping exercise.
- a stepping stone toward full SLAM.
- a portfolio nano-project showing practical understanding of robot perception pipelines.

The Python node currently implements a basic occupancy-marking mapper, preserving a very direct scan-to-grid pipeline rather than a heavier inference layer.

## 3. Mathematical Model

### 3.1 Robot Pose

The robot pose is represented as:

$$
\mathbf{x}_r = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
$$

where:
- $x$ = robot position in world frame along the x-axis.
- $y$ = robot position in world frame along the y-axis.
- $\theta$ = robot heading angle (yaw).

The node reads orientation from odometry as a quaternion and converts it into yaw using Euler angle extraction.

### 3.2 Laser Point in Robot Frame

For each LiDAR range reading $r_i$ at angle $\alpha_i$, the corresponding obstacle point in the robot frame is:

$$
x_r^{(i)} = r_i \cos(\alpha_i)
$$

$$
y_r^{(i)} = r_i \sin(\alpha_i)
$$

This converts polar scan data into Cartesian coordinates relative to the robot. The implementation does this directly in the scan callback.

### 3.3 Transform from Robot Frame to World Frame

Given robot pose $(x, y, \theta)$, each scan point is transformed into the world frame as:

$$
x_w^{(i)} = x + x_r^{(i)}\cos\theta - y_r^{(i)}\sin\theta
$$

$$
y_w^{(i)} = y + x_r^{(i)}\sin\theta + y_r^{(i)}\cos\theta
$$

This is a standard rigid-body planar transformation. The node applies exactly this mapping before projection into the grid.

### 3.4 World Coordinates to Grid Coordinates

Let the occupancy grid have:
- resolution $r_{map}$ in meters/cell.
- origin $(x_0, y_0)$.

Then world coordinates are mapped into grid indices by:

$$
m_x = \left\lfloor \frac{x_w - x_0}{r_{map}} \right\rfloor
$$

$$
m_y = \left\lfloor \frac{y_w - y_0}{r_{map}} \right\rfloor
$$

If the indices lie within the map bounds, the corresponding cell is marked occupied:

$$
\text{map}[m_y, m_x] = 100
$$

The current node uses:
- resolution = `0.05 m/cell`.
- width = `200`.
- height = `200`.
- origin = `[-10.0, -10.0]`.

which corresponds to a 10 m × 10 m map centered around the working region.

## 4. Implementation Details

The package is structured as a ROS1 catkin package named `scan2grid_tb3`, with dependencies on standard ROS message packages, `tf`, and TurtleBot3 simulation components. The Python node is installed through `catkin_install_python`, and the `launch/` directory is installed along with the package.

### Mapping node behavior

The `basic_slam.py` node performs the following:
- initializes a ROS node.
- subscribes to `/odom`.
- subscribes to `/scan`.
- stores robot pose as $[x, y, \theta]$.
- initializes a 2D occupancy grid with unknown values `-1`.
- converts valid scan measurements into occupied map cells.
- publishes the map periodically as `OccupancyGrid`.

### Current mapping policy

The current map update rule is deliberately simple:

- invalid scan values are skipped.
- valid in-range scan points are transformed.
- only obstacle endpoints are marked as occupied.
- free space is not explicitly ray-traced.
- probabilistic updates are not used.

So this project is best described as **basic occupancy-grid mapping**, not a full modern SLAM system.

## 5. Folder Structure

```text
Scan2Grid-TB3/
├── assets/
│   └── demo.mp4
├── launch/
│   └── basic_slam.launch
├── scripts/
│   └── basic_slam.py
├── CMakeLists.txt
├── package.xml
├── README.md
└── LICENSE
```

### File descriptions

- `scripts/basic_slam.py`  
  Main ROS1 Python node for occupancy-grid map generation.

- `launch/basic_slam.launch`  
  Launch file that starts the TurtleBot3 world, mapping node, and RViz.

- `CMakeLists.txt`  
  Catkin build configuration with Python node installation and launch directory installation.

- `package.xml`  
  Package metadata, dependencies, maintainer, repository URL, and license.

- `assets/demo.mp4`  
  Demonstration video of the mapping process.

## 6. How to Execute

### 6.1 Prerequisites

This project assumes:
- Ubuntu with ROS1 Noetic.
- a catkin workspace.
- TurtleBot3 packages installed.
- Gazebo and RViz available.

Install dependencies:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-turtlebot3-gazebo
sudo apt-get install ros-noetic-slam-gmapping
```

> Note: `slam_gmapping` is included as a runtime dependency in the package metadata, though the current node itself publishes its own occupancy map rather than invoking gmapping directly.

### 6.2 Build the Package

Place the package inside the `src/` folder of your catkin workspace and run:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

The package is configured as a ROS1 catkin package named `scan2grid_tb3`.

### 6.3 Run the Project

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch scan2grid_tb3 basic_slam.launch
```

This launches:
- the TurtleBot3 Gazebo world.
- the custom `basic_slam.py` node.
- RViz for visualization.

During execution, the robot can be teleoperated to observe environment coverage and progressive map formation.

## 7. How to Re-Implement

To rebuild this project from scratch, follow this roadmap.

### Step 1: Create a ROS1 catkin package

Create a package with dependencies:

- `rospy`
- `roscpp`
- `sensor_msgs`
- `nav_msgs`
- `std_msgs`
- `tf`

This matches the current package dependency configuration.

### Step 2: Initialize the ROS node

Create a Python script and initialize a ROS node:

```python
rospy.init_node("basic_slam", anonymous=True)
```

### Step 3: Subscribe to odometry and laser scan topics

Set up subscribers for:
- `/odom` using `Odometry`.
- `/scan` using `LaserScan`.

Store robot pose as $[x, y, \theta]$.

### Step 4: Build the grid map

Initialize a NumPy array with shape `(height, width)` using unknown value `-1`.

Example:

```python
self.map = -np.ones((self.map_height, self.map_width), dtype=np.int8)
```

The current implementation uses:
- width = 200.
- height = 200.
- resolution = 0.05.
- origin = [-10.0, -10.0].

### Step 5: Convert scan rays into Cartesian coordinates

For each valid range reading:
- compute local point $(x_r, y_r)$.
- transform into world coordinates $(x_w, y_w)$.
- convert to map indices.
- mark that cell as occupied.

That is the essential scan-to-grid pipeline in this project.

### Step 6: Publish `OccupancyGrid`

Populate:
- header.
- resolution.
- width.
- height.
- origin.
- flattened grid data.

Then publish the map periodically.

### Step 7: Add a launch file

Create a ROS launch file that:
- sets TurtleBot3 model.
- includes TurtleBot3 Gazebo world launch.
- launches your mapping node.
- launches RViz.

### Step 8: Build, source, and launch

Compile with `catkin_make`, source the workspace, and run with `roslaunch`.

## 8. Output / Result

The output is a **2D occupancy map** representing detected obstacles in the TurtleBot3 environment. The attached result shows that the system successfully reconstructs the hexagonal obstacle arrangement and the surrounding wall-like structure in RViz while the robot explores the scene.

Observed characteristics from the result:
- the outer obstacle boundary is clearly recovered.
- the internal pillar-like obstacle points are also captured.
- the robot's explored region is reflected in the occupancy grid.
- the map geometry qualitatively matches the Gazebo scene layout.

This indicates that the scan-to-world and world-to-grid transformations are functioning correctly, and that the occupancy mapping pipeline is internally coherent for the environment shown.

## 9. Conclusion

Scan2Grid-TB3 demonstrates a compact, transparent, and educational occupancy-grid mapping pipeline for TurtleBot3 in ROS1. By using odometry and LiDAR directly, the project exposes the geometric core of robotic mapping without hiding the logic inside a heavyweight SLAM framework.

### What this project achieves

- practical ROS1 node development.
- handling of odometry and laser scan messages.
- coordinate transformation from sensor frame to world frame.
- discretization into an occupancy grid.
- simulation-based validation in Gazebo and RViz.

### What it does not yet include

- free-space ray tracing.
- probabilistic map updates.
- scan matching.
- loop closure.
- global optimization.

So the project sits in a very useful zone:

**more meaningful than a toy subscriber demo, but intentionally simpler than full SLAM.**

It is best viewed as a strong nano-project in robotics perception and a clean stepping stone toward richer mapping and localization systems.

## 10. License

> **MIT License**

---