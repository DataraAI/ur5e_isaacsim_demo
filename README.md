# UR5e Isaac Sim demo

ROS 2 Jazzy workspace that launches a UR5e into NVIDIA Isaac Sim and brings up MoveIt against the simulator's ROS 2 bridge.

Isaac Sim is expected at `~/isaacsim` (this setup was run with Isaac Sim 6.0). The ROS side uses the official Universal Robots description and MoveIt config. Joint commands and joint states cross the bridge on `/isaac_joint_commands` and `/isaac_joint_states`, with simulation time on `/clock`.

## Packages

| Package | Role |
|---|---|
| `ur_description` | Official URDF, meshes, and kinematics from [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), branch `jazzy`. |
| `ur_moveit_config` | Official MoveIt config. `Universal_Robots_ROS2_MoveIt_Config` is no longer a separate repository; the package lives in [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), branch `jazzy`. This workspace keeps a sparse checkout of `ur_moveit_config` only. |
| `topic_based_ros2_control` | ros2_control hardware interface from [PickNikRobotics/topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control). It publishes joint commands and subscribes to joint states so MoveIt can drive the simulated arm. |
| `ur5e_isaac_bridge` | Launch package for this demo. It imports the UR5e URDF into Isaac Sim, creates the ROS 2 bridge action graph, and starts `ros2_control` and MoveIt after `/clock` and `/isaac_joint_states` are publishing. |

Upstream pins are listed in `ros2_ws/ur5e_isaac.repos`.

## Data center assets

The Data Hall asset stage and its components are in NVIDIA's Data Center Assets Pack.

1. Open the [Data Center Assets Pack](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html#data-center-assets-pack) section of the Omniverse downloadable packs page.
2. Click **Download the Data Center Assets Pack**.

That downloads a ZIP of the USD assets, including the Data Hall stage and components. The pack is about 9.8 GB.

Unzip the pack and open `Stages/Data_Hall/DataHall_Full_B01.usd`. That is the stage to use. It contains the data hall, the rack row, and the equipment placed in those racks. Physics and collision are not authored on this stage, so nothing in it collides until you add colliders yourself.

Each rack has a `Front_Door` that is invisible by default. The door mesh is authored with zero opacity, and on some racks the prim visibility is set to invisible as well. The door is still in the stage. If you enable collision on the racks, that hidden door can become a collider and block the arm even though you cannot see it.

Deactivate every `Front_Door` before you add collision, so the door is not part of the stage:

1. In Isaac Sim, open `DataHall_Full_B01.usd`.
2. In the Stage panel, search for `Front_Door`.
3. There is one under each rack, at `/World/DataHall_01/DataHall_01/DataHall_Racks/Rack_42U_0N/Rack_42U_01/Front_Door` (`N` is 1 through 5). Select the `Front_Door` xform, not only a child such as `Front_Door_Frame`.
4. Right-click it and choose **Deactivate**.

A deactivated prim is dropped from the composed stage, so it is neither drawn nor given a collider later. Hiding it with the eye icon only changes visibility. Do that deactivate step first, then add collision to the hall.

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`topic_based_ros2_control` configures tests that need `ros-jazzy-ros-testing`. The `-DBUILD_TESTING=OFF` argument skips those test-only dependencies.

## Launch

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py
```

The launch file starts Isaac Sim with `~/isaacsim/python.sh`, imports the UR5e, and presses Play inside the standalone script. MoveIt and RViz start after the bridge topics appear. In RViz, plan with the `ur_manipulator` group.

Useful arguments:

| Argument | Default | Meaning |
|---|---|---|
| `ur_type` | `ur5e` | Robot model passed to the Universal Robots description. |
| `launch_isaac` | `true` | Start Isaac Sim from this launch. Set `false` to attach MoveIt to a simulator that is already running. |
| `launch_rviz` | `true` | Start RViz with the official MoveIt display config. |
| `use_sim_time` | `true` | Use `/clock` from Isaac Sim. |
| `headless` | `false` | Run the Isaac Sim app without a window. |
| `isaac_python` | `~/isaacsim/python.sh` | Isaac Sim Python launcher. |
| `joint_commands_topic` | `/isaac_joint_commands` | Commands from ros2_control into Isaac Sim. |
| `joint_states_topic` | `/isaac_joint_states` | Joint states from Isaac Sim into ros2_control. |

The first Isaac Sim startup can take longer than the launch's 180 second wait. If the log reports that the bridge timed out and MoveIt was not launched, leave the simulator running and start MoveIt against it:

```bash
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py launch_isaac:=false
```

The ROS terminal and Isaac Sim need the same `ROS_DOMAIN_ID`. The bridge graph uses the domain from that environment variable.
