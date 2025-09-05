# Hector Quadrotor Repository – Custom Modifications

This repository is a fork of the original [Hector Quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) project. The modifications made here are for algorithms testing which focus on GNC simulation in Gazebo.

---

## Overview of Work

The main objective of the work done in this repository was to improve UAV simulation capabilities, specifically in:

- **Navigation & Guidance:** Enhanced flight behaviors including takeoff, landing, and waypoint navigation.
- **Sensor Integration:** Customized sensor models for cameras, LiDAR (Hokuyo), depth sensors, and GNSS.
- **Simulation Environments:** Added new world files for testing UAV behavior in varied environments.
- **UI Improvements:** Modified the UI scripts to better visualize and control leader-follower drone setups.

---

## Key Changes

### Configuration Files
- **`GNSS.cfg` & `SensorModel.cfg`**  
  Custom sensors model 

### UAV Sensor URDFs
- Modified URDF/XACRO files for:
  - Generic camera (`generic_camera.urdf.xacro`)
  - Hokuyo LiDAR sensors (`hokuyo_utm30lx.urdf.xacro`, `hokuyo_utm31lx.urdf.xacro`)
  - Depth cameras (`depth.urdf.xacro`)  
  Custom sensor placement, orientation, and parameters for navigation algorithms.

### Launch Files
- Modified launch files under `hector_quadrotor_gazebo/launch`:
  - **`camera_launch.launch`** – For initializing custom camera sensors.
  - **`quadrotor_empty_world.launch`** – Empty world for initial testing purposes
  - **`takeoff_land.launch`** – Customized takeoff and landing behaviors.

### UI Scripts
- Updated Python scripts for the user interface:
  - `ui_hector_quad.py`
  - `ui_hector_quad_leader.py`
  - `ui_hector_quad_follower.py`  
    Focus on visualizing leader-follower drone formations based on velocities  

### Takeoff & Landing Logic
- **`takeoff_land_code.py`** – Implemented custom routines for automated takeoff, hover, and landing sequences. 

### Custom World Files
- Added several new Gazebo worlds (`autoware1.world`, `custom1.world`, `custom2.world`, `custom_world.world`) for testing navigation in different environments.

---

