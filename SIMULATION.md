# LiBot simulation and web control

The saved ROS map is the coordinate source for Gazebo and the web UI. Regenerate both artifacts after changing the PGM or YAML:

```bash
python3 tools/generate_map_assets.py \
  --map maps/library_map.yaml \
  --world ros2_ws/src/libot_description/worlds/library.generated.sdf \
  --scene ../Libbot_FE/public/maps/library-map.json
```

## Run the complete simulation

Start the .NET API on port 5205 and the Vite frontend on port 5173, then run.
The simulation publishes its command gateway on host port `9001` and reaches the
API through `host.docker.internal`, which works on Docker Desktop and Linux via
the Compose `host-gateway` mapping.

```bash
cd Libbot_Robot
docker compose -f docker/compose.yaml --profile simulation up -d --build
```

Open `http://localhost:5173/map-demo`, select **SIM**, set the initial pose, wait for `LOCALIZED`, then select and confirm a destination.

Gazebo simulation uses its ground-truth odometry for `map -> odom`; AMCL remains
available for pose estimation but does not publish that TF in SIM. The hardware
profile continues to use AMCL as the only `map -> odom` publisher.

Useful smoke checks inside the simulation container:

```bash
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

In the web UI select **REAL**, arm the robot, set its initial pose, and only test navigation after AMCL reports `LOCALIZED`. The web stop button cancels Nav2 and publishes zero velocity; it is not a replacement for the physical emergency stop.

