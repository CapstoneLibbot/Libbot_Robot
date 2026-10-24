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

## Run the complete simulation

Generate the layout, start the .NET API on port 5205 and the Vite frontend on port 5173, then rebuild and start the simulation image. Rebuilding includes the new SDF world in the ROS install tree; the generated Nav2 map is mounted from `maps/`.

```bash
cd /mnt/d/Capstone/Project/Robot/libot_pi
python3 tools/generate_library_layout.py
docker compose -f docker/compose.yaml --profile simulation up -d --build
```

On the FE, open `/maps/1` for the 3D layout preview. To watch a workflow, open `/tasks` and select a mission in **Robot missions**. A mission remains queued until **Run** is pressed. Failed or cancelled missions can be explicitly retried with **Run again**. The monitor keeps workflow steps on the left and the map on the right; use **Gazebo 3D** and **2D map** to switch views. Both show the robot at its current telemetry pose, refreshed every 250 ms, with the reading-room and toilet labels; the workflow panel highlights active, completed, and failed steps. For a SIM mission, the controls at the upper right set Gazebo to x1, x2, x3, or x4. They speed up Gazebo time, Nav2 movement, and duration/context WAIT step deadlines in SIM. User-input waits keep real time. The controls do not alter the physical robot. Actual acceleration may be limited by host CPU capacity. The frontend uses `http://localhost:5205` by default; set `VITE_API_BASE_URL` if the backend is hosted elsewhere.

The simulation publishes its command gateway on host port `9001` and reaches the API through `host.docker.internal`, which works on Docker Desktop and Linux with the Compose `host-gateway` mapping.

Gazebo simulation uses ground-truth odometry for `map -> odom`; AMCL remains available for pose estimation but does not publish that transform in SIM. The hardware profile continues to use AMCL as the only `map -> odom` publisher.

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

## Run the physical robot

On the Raspberry Pi, set the backend URL to the PC's LAN address:

```bash
LIBOT_BACKEND_URL=http://192.168.1.10:5205 \
docker compose -f docker/compose.yaml --profile hardware up --build
```

In the web UI select **REAL**, arm the robot, set its initial pose, and test navigation only after AMCL reports `LOCALIZED`. The web stop button cancels Nav2 and publishes zero velocity; it is not a replacement for the physical emergency stop.
