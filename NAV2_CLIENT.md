# Nav2 fixed-map client

The ROS containers use the same local Fast DDS settings as the reference workflow:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=7
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
ls -lh /ros2_ws/maps/library_map.*
```

The container entrypoint applies these ROS settings for launched processes. The map directory is mounted at `/ros2_ws/maps`.

## Select the map

`navigation.launch.py` accepts the `map` launch argument. Its default is `/ros2_ws/maps/library_map.yaml`. To use a different saved map, pass that map's YAML path:

```bash
ros2 launch libot_bringup navigation.launch.py \
  map:=/ros2_ws/maps/my_saved_map.yaml \
  use_hardware:=false \
  start_workflow_app:=false \
  autostart_navigation:=true
```

Keep the YAML and its image together. The YAML image entry should name the companion PGM, for example `image: your_map.pgm`. Do not change `nav2.yaml` to switch maps; the launch argument selects the map.
Replace `my_saved_map.yaml` with the filename of the map you want to load. Stop the existing mapping/navigation launch before starting a different map.
For a physical robot, start the hardware profile once so `robot_control.launch.py` owns the LiDAR, STM32 bridge and TF; `use_hardware:=false` prevents this Nav2 launch from starting duplicate hardware nodes.

The hardware control page can also start Nav2 from the map selected in the map UI. The backend passes that selected YAML path to the robot and checks that it exists under `/ros2_ws/maps`.

For the LiBot Gazebo world, the default map is `/ros2_ws/maps/library_sim.yaml`. To show Gazebo and RViz while running the standalone reference client:

```bash
ros2 launch libot_bringup simulation_navigation.launch.py \
  map:=/ros2_ws/maps/library_sim.yaml \
  headless:=false use_rviz:=true autostart_navigation:=true
```

Use a map generated for the same world and coordinate frame as the simulated robot. The physical `library_map.yaml` and simulation `library_sim.yaml` describe different maps.

## Check the reference stack

Stop the mapping launch before starting fixed-map navigation. Start the launch with the desired map, then check that the map, AMCL, TF and Nav2 action server are available:

```bash
ros2 node list | sort
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
```

`base_footprint` is the base frame configured for LiBot Nav2. The URDF also provides `base_footprint -> base_link`, so `tf2_echo map base_link` is available when you specifically need that frame.

## Send one goal

The client file is `code/navigate_goal.py`. It uses the map frame for the initial and goal poses, stamps them with ROS time, converts yaw radians to a quaternion, waits for Nav2, handles missing feedback, and starts the active-goal timeout after sending the goal. A timeout requests cancellation and keeps waiting for the terminal action result.

On the robot container, after the reference-stack checks pass:

```bash
cd /ros2_ws
python3 code/navigate_goal.py --help
python3 code/navigate_goal.py \
  --initial-x 1.0 --initial-y 2.0 --initial-yaw 0.0 \
  --goal-x 3.0 --goal-y 4.0 --goal-yaw 1.57 \
  --timeout 90
```

Replace the sample coordinates with measured poses in the currently loaded map. X/Y use meters and yaw uses radians. Use `--skip-initial-pose` only when localization has already been established, such as the Gazebo ground-truth workflow.

If the cancellation request does not produce a terminal result within the configured cancellation wait, the client reports that explicitly and exits with status `124`; it does not report a successful navigation.
