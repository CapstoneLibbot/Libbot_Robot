# Chạy mapping LiBot

Không chạy riêng `description.launch.py`, `sllidar_a1_launch.py`,
`static_transform_publisher` hoặc `slam_toolbox` cùng lúc với launch bên dưới.
`mapping.launch.py` đã khởi động tất cả các node cần thiết. Chạy trùng node sẽ
tạo nhiều TF publisher và có thể làm bản đồ bị lệch/chồng lên nhau.

```bash
cd ~/Libbot/Libbot_Robot
docker exec -it libot_ros2 bash

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select \
  libot_description libbot_stm32 libot_bringup
source install/setup.bash

ros2 launch libot_bringup mapping.launch.py \
  lidar_port:=/dev/rplidar \
  stm32_port:=/dev/serial0
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