ros-dds-bridge
=================

Small ROS1 <-> DDS bridge framework (architecture-focused)

Overview
--------
This repository hosts a small bridge node that converts messages between ROS (roscpp) and DDS (eProsima Fast DDS).
The current work focuses on a clean architecture so additional bridge implementations (in particular DDS->ROS) can be added easily.

Key design points
-----------------
- Two clear bridge directions and directories:
  - `src/ros_to_dds/` — bridges that subscribe to ROS topics and publish DDS samples.
  - `src/dds_to_ros/` — bridges that subscribe to DDS topics and publish ROS messages (future work / placeholder present).
- Direction-prefixed concrete bridge classes to avoid name collisions. Example: `ros_to_dds::RosToDdsPoseBridge`.
- `BridgeManager` centralizes creation of (shared) DDS resources (DomainParticipant and Publisher). It manages the lifecycle and initializes registered bridges.
- Bridges accept a `set_dds_context(...)` call so they can reuse a shared participant/publisher provided by `BridgeManager`, or create their own participant/publisher as fallback.

Files of interest
-----------------
- `src/BridgeManager.hpp/.cpp` — manages bridge registration and shared DDS context.
- `src/RosDdsBridge.cpp` — `main()` that creates `BridgeManager` and registers bridges.
- `src/ros_to_dds/` — ROS->DDS bridge interfaces and implementations. Example: `RosToDdsPoseBridge`.
- `src/dds_to_ros/` — DDS->ROS bridge interfaces (skeleton) and place to add implementations later.
- `CMakeLists.txt` — cmake build. The node is built as `RosDdsBridge`.

How to add a new bridge
-----------------------
1. Decide the direction:
   - ROS->DDS: create a new class deriving from `ros_to_dds::RosToDdsBridge`.
   - DDS->ROS: create a new class deriving from `dds_to_ros::DdsToRosBridge`.
2. Implement `ros_topic()`/`dds_topic()` and `init()` plus `set_dds_context(...)` if you want to receive the shared participant/publisher/subscriber.
3. In `src/RosDdsBridge.cpp` register your bridge using:
   - `mgr.register_ros_to_dds_bridge(...)` or
   - `mgr.register_dds_to_ros_bridge(...)`.
4. Rebuild the package.

Notes / TODOs
------------
- DDS->ROS implementations are intentionally left for later. Current structure supports adding them with minimal friction.
- Decide whether `BridgeManager` should create a shared `Subscriber` for DDS->ROS bridges. Right now DDS->ROS bridges can create their own Subscriber.
- There are a few places where `ROBOT_ID` environment variable is used to name participants. Keep that in mind when running multiple bridge nodes.

Building
--------
This is a catkin-style package (ROS1). Typical build (from workspace root):

```bash
# from containing catkin workspace
catkin_make
# or, using colcon/catkin_tools, run the appropriate build command
```
Running
-------
Set `ROBOT_ID` before launching (some code paths require it when creating a shared participant):

```bash
export ROBOT_ID=my_robot
rosrun ros_dds_bridge RosDdsBridge
```

If you prefer to use the included `ros_dds_bridge.launch`, use `roslaunch`.
