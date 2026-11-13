# LiBot simulation and web control

The simulation library is 50 m wide on X and 60 m long on Y. One layout file generates the Gazebo world, the simulation-only Nav2 map, the FE map assets, and the static 3D preview:

```bash
python3 tools/generate_library_layout.py
```

The source is `ros2_ws/src/libot_description/config/library_layout.json`. The generated room has a 3 m south entrance, four bookshelf units, a librarian counter, a partitioned reading room, and a toilet with fixtures. The room remains 50 × 60 m. The physical robot map `maps/library_map.yaml` and the existing FE editor map are separate and are not replaced.

## Open the world in WSL

Use Ubuntu 24.04 on WSL 2 with WSLg enabled. Install ROS 2 Jazzy and Gazebo Harmonic using the [ROS Jazzy instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and [Gazebo Harmonic ROS instructions](https://gazebosim.org/docs/harmonic/ros_installation/). Install the Gazebo ROS integration if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
source /opt/ros/jazzy/setup.bash
```

Regenerate the assets and open the SDF world:

```bash
cd /mnt/d/Capstone/Project/Robot/libot_pi
python3 tools/generate_library_layout.py
gz sim ros2_ws/src/libot_description/worlds/library.sdf
```

To see the static 3D preview, run `npm run dev` from `FE`, open `/maps/1`, and choose **Gazebo 3D**. Drag to rotate and use the mouse wheel to zoom. The preview shows the floor plan; Gazebo adds the simulated robot when its runtime is running.

## Simulate a saved Nav2 map

The backend occupancy grid can be converted into a Gazebo SDF world. The generator uses the map YAML's image, resolution, origin, and occupancy thresholds; occupied cells become static collision boxes. Unknown regions enclosed by occupied cells are filled as solid obstacles, while open unknown space is left clear.

```bash
python3 tools/generate_gazebo_world_from_map.py \
  --map maps/backend_bb72855a2eab4d98864df0b1d4fe7ec8.yaml
```

For this map, the robot spawn point below is in free space. From the `Libbot_Robot` directory, run the full simulation with the same map loaded by Nav2 and its generated Gazebo world:

```bash
docker compose -f docker/compose.yaml --profile simulation run --build --rm --service-ports ros2-simulation \
  ros2 launch libot_bringup simulation_navigation.launch.py \
  headless:=false use_rviz:=true autostart_navigation:=true \
  world:=/ros2_ws/src/libot_description/worlds/backend_bb72855a2eab4d98864df0b1d4fe7ec8_gazebo.sdf \
  map:=/ros2_ws/maps/backend_bb72855a2eab4d98864df0b1d4fe7ec8.yaml \
  spawn_x:=0.807 spawn_y:=-0.074 spawn_z:=0.15 spawn_yaw:=0.0
```

`spawn_x`, `spawn_y`, and `spawn_yaw` are map/world coordinates in metres and radians. Choose a different free cell if the physical robot's initial pose is elsewhere. The generated world represents the 2D occupancy grid as 2 m tall obstacles; it does not recover real obstacle heights or shapes from the map.

### Compare Gazebo truth with Nav2's estimate in RViz

The simulation bridges the Gazebo model-specific pose topic to `/gazebo/ground_truth_pose`. In RViz, choose **Add → Pose**, set **Topic** to `/gazebo/ground_truth_pose`, and keep **Fixed Frame** set to `map`. The Pose arrow shows Gazebo's physical pose. The RobotModel uses Nav2's AMCL localization, initialized from the configured spawn pose and corrected from LiDAR scans. These should align when the map, world, and spawn pose match.

If the Pose arrow is not visible, check that `/gazebo/ground_truth_pose` is publishing with `ros2 topic hz /gazebo/ground_truth_pose`. The world/map pair must use the same coordinates, as they do for the generated backend map world.

## Run the complete simulation

Generate the layout, start the .NET API on port 5205 and the Vite frontend on port 5173, then rebuild and start the simulation image. Rebuilding includes the new SDF world in the ROS install tree; the generated Nav2 map is mounted from `maps/`.

```bash
cd /mnt/d/Capstone/Project/Robot/libot_pi
python3 tools/generate_library_layout.py
docker compose -f docker/compose.yaml --profile simulation up -d --build
```

On the FE, open `/maps/1` for the 3D layout preview. To watch a workflow, open `/tasks` and select a mission in **Robot missions**. A mission remains queued until **Run** is pressed. Failed or cancelled missions can be explicitly retried with **Run again**. The monitor keeps workflow steps on the left and the map on the right; use **Gazebo 3D** and **2D map** to switch views. Both show the robot at its current telemetry pose, refreshed every 250 ms, with the reading-room and toilet labels; the workflow panel highlights active, completed, and failed steps. For a SIM mission, the controls at the upper right set Gazebo to x1, x2, x3, or x4. They speed up Gazebo time, Nav2 movement, and duration/context WAIT step deadlines in SIM. User-input waits keep real time. The controls do not alter the physical robot. Actual acceleration may be limited by host CPU capacity. The frontend uses `http://localhost:5205` by default; set `VITE_API_BASE_URL` if the backend is hosted elsewhere.

The simulation publishes its command gateway on host port `9001` and reaches the API through `host.docker.internal`, which works on Docker Desktop and Linux with the Compose `host-gateway` mapping.

Gazebo simulation and the hardware profile both use AMCL to publish `map -> odom`. In simulation, AMCL starts from the configured spawn pose and corrects it using LiDAR scans. `/gazebo/ground_truth_pose` is a read-only comparison display; Nav2 does not consume it.

The simulated robot uses Gazebo Sim's mecanum drive system with four driven
wheels. Nav2 velocity commands pass through Collision Monitor before the Gazebo
bridge, using `/cmd_vel_safe` as the simulator input. MPPI can command forward,
lateral, and rotational velocity; the simulation wheel contacts approximate
the diagonal roller friction of the physical mecanum wheels.
Some Jazzy binary releases do not export `OmniConstraintCritic` and
`MecanumCritic`; `libot_mppi_critics` provides them only when they are missing
from the installed MPPI package.

Useful checks inside the simulation container:

```bash
docker exec -it libot_ros2_simulation bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /bt_navigator
```

### Diagnose a goal that aborts

If RViz shows a global path but the goal aborts, check the velocity pipeline while a new goal is active. Run each command in a second container shell; `--once` should return promptly while the robot is trying to move:

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /collision_monitor_state --once
ros2 topic echo /odom --once
```

`/cmd_vel_nav` is the controller output, `/cmd_vel` is the velocity smoother output, and `/cmd_vel_safe` is the Collision Monitor output sent to Gazebo. A nonzero command that becomes zero identifies the stage suppressing motion; nonzero `/cmd_vel_safe` with a stationary Gazebo model points to the Gazebo bridge or mecanum drive. If `/cmd_vel_nav` itself is zero, inspect `controller_server` logs and the local costmap. Also check the Collision Monitor's `FootprintApproach` state: it currently uses `min_points: 1`, so a single LiDAR return in the footprint can trigger its approach limit in this tight map.

## Run the physical robot

On the Raspberry Pi, set the backend URL to the PC's LAN address:

```bash
LIBOT_BACKEND_URL=http://192.168.1.10:5205 \
docker compose -f docker/compose.yaml --profile hardware up --build
```

In the web UI select **REAL**, arm the robot, set its initial pose, and test navigation only after AMCL reports `LOCALIZED`. The web stop button cancels Nav2 and publishes zero velocity; it is not a replacement for the physical emergency stop.
