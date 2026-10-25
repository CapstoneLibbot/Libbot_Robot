# Chạy mapping LiBot

## Giao diện điều khiển robot thật

`docker compose --profile hardware up -d --build` khởi động `robot_control.launch.py`:
robot app, LiDAR, STM32 và TF; SLAM/Nav2 được mở riêng từ trang FE `/robot-mapping`.
Trang FE gửi lệnh lái, xem map `/map`, lưu PGM/YAML trên robot cùng snapshot database,
và khởi chạy Nav2 từ map đã chọn. Không chạy thêm launch phần cứng trong container này.

Không chạy riêng `description.launch.py`, `sllidar_a1_launch.py`,
`static_transform_publisher` hoặc `slam_toolbox` cùng lúc với launch bên dưới.
`mapping.launch.py` đã khởi động tất cả các node cần thiết. Chạy trùng node sẽ
tạo nhiều TF publisher và có thể làm bản đồ bị lệch/chồng lên nhau.

```bash
cd ~/Libbot/Libbot_Robot
docker exec -it libot_ros2_hardware bash

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install \
  --base-paths src/libot_description src/libbot_stm32 src/libot_bringup \
  --packages-select libot_description libbot_stm32 libot_bringup
source install/setup.bash

ros2 launch libot_bringup mapping.launch.py \
  lidar_port:=/dev/rplidar \
  stm32_port:=/dev/serial0 \
  use_hardware:=false
```

Trong RViz dùng `Fixed Frame: map`, sau đó thêm `/map` và `/scan`.

## Kiểm tra TF trước khi quét

Cây TF hợp lệ phải chỉ có một chuỗi:

```text
map -> odom -> base_footprint -> base_link -> laser
```

Kiểm tra node/TF bị chạy trùng:

```bash
ros2 node list | sort
ros2 run tf2_tools view_frames
ros2 topic hz /scan
ros2 topic hz /odom
```

Không chạy lệnh TF tĩnh `odom -> base_footprint`: transform này do
`stm32_bridge` phát động từ encoder.




ros2 run nav2_map_server map_saver_cli \
  -f /ros2_ws/maps/library_map \
  --ros-args -p map_subscribe_transient_local:=true

## Chạy Nav2 và gửi goal trực tiếp từ RViz

Không chạy `mapping.launch.py` cùng lúc với navigation vì hai launch file sẽ tạo
trùng LiDAR, STM32 bridge và robot state publisher.

```bash
cd ~/Libbot/Libbot_Robot
docker compose -f docker/compose.yaml --profile hardware up -d --build
docker exec -it libot_ros2_hardware bash

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install \
  --base-paths src/libot_description src/libbot_stm32 src/libot_bringup \
  --packages-select libot_description libbot_stm32 libot_bringup
source install/setup.bash

ros2 launch libot_bringup navigation.launch.py \
  map:=/ros2_ws/maps/library_map.yaml \
  lidar_port:=/dev/rplidar \
  stm32_port:=/dev/serial0 \
  use_hardware:=false \
  start_workflow_app:=false \
  autostart_navigation:=true
```

Khi RViz mở:

1. Chờ bản đồ, robot và dữ liệu laser xuất hiện.
2. Chọn **2D Pose Estimate**, bấm vào vị trí thật của robot trên bản đồ rồi kéo
   mũi tên theo đúng hướng robot đang quay.
3. Chờ laser khớp với tường trên bản đồ.
4. Chọn **Nav2 Goal**, bấm vào vị trí đích và kéo mũi tên theo hướng dừng mong
   muốn.

Kiểm tra nhanh trong terminal khác nếu robot không chạy:

```bash
docker exec -it libot_ros2_hardware bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /bt_navigator
```

Để chỉ chạy Nav2/RViz khi phần cứng đã được khởi động bởi một launch khác, thêm
`use_hardware:=false`. Để chạy không có giao diện, thêm `use_rviz:=false`.
