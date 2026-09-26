Terminal 1 - Robot State Publisher


cd ~/Libbot/Libbot_Robot
docker exec -it libot_ros2 bash

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch libot_description description.launch.py
=================================================================
Terminal 2 — RPLIDAR


docker exec -it libot_ros2 bash

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch sllidar_ros2 sllidar_a1_launch.py \
    serial_port:=/dev/rplidar \
    serial_baudrate:=115200 \
    frame_id:=laser
=================================================================
Terminal 4 — tạo odom -> base_footprint tạm thời


docker exec -it libot_ros2 bash


source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 \
    odom base_footprint

=================================================================
Terminal 5 — SLAM Toolbox


docker exec -it libot_ros2 bash

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py

=================================================================
Terminal 6 — RViz

docker exec -it libot_ros2 bash


source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

rviz2

=============================================
Cấu hình RViz

Trong RViz:

1. Fixed Frame

Ở panel bên trái:

Global Options
    Fixed Frame: map
2. Add Map

Bấm:

Add

→ chọn:

By topic
→ /map
→ Map
3. Add LaserScan

Bấm:

Add
→ By topic
→ /scan
→ LaserScan

Đặt:

Topic: /scan

và:

Reliability: Best Effort