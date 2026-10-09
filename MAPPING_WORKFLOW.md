# LiBot Mapping & SLAM Workflow

Tài liệu này ghi lại toàn bộ workflow cần thực hiện để triển khai tính năng `mapping` cho robot LiBot sử dụng ROS 2 + SLAM Toolbox + Lidar 2D + STM32.

---

## 1. Mục tiêu

Mục đích là cho robot:
- di chuyển trong không gian nội thất,
- quét môi trường bằng Lidar 2D,
- tạo bản đồ 2D bằng SLAM Toolbox,
- lưu map thành file `.yaml` và `.pgm` để dùng cho localization sau này.

---

## 2. Kiến trúc hệ thống

### 2.1 ROS 2 workspace

Cấu trúc hiện tại:

- `ros2_ws/src/libot_bringup`
  - file launch
  - file config YAML
  - file RViz
  - thư mục `maps`
- `ros2_ws/src/libot_description`
  - URDF robot
- `ros2_ws/src/libot_stm32`
  - bridge ROS ↔ STM32

### 2.2 Thành phần cần có

- `robot_state_publisher`
- `slam_toolbox`
- `rviz2`
- `teleop_twist_keyboard`
- `rplidar_ros` hoặc driver lidar tương ứng
- `libot_stm32` bridge

---

## 3. Package cần thêm / cài đặt

### 3.1 Package ROS 2 cần có

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-rviz2
sudo apt install ros-humble-rplidar-ros
```

Nếu cần lưu map bằng `nav2_map_server`:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-map-server
```

### 3.2 Package trong workspace

- `libot_description`
- `libot_stm32`
- `libot_bringup`

---

## 4. Vị trí file cấu hình

### 4.1 Launch file

Nên đặt ở:

```text
ros2_ws/src/libot_bringup/launch/
```

Ví dụ:

- `mapping.launch.py`

### 4.2 YAML config

Nên đặt ở:

```text
ros2_ws/src/libot_bringup/config/
```

Ví dụ:

- `mapper_params_online_async.yaml`
- `slam.rviz`

### 4.3 Map output

Nên lưu ở:

```text
ros2_ws/src/libot_bringup/maps/
```

Ví dụ:

- `libot_map.yaml`
- `libot_map.pgm`

---

## 5. Cấu hình SLAM Toolbox

File tham số nên bao gồm các thông số chính:

- `odom_frame: odom`
- `map_frame: map`
- `base_frame: base_footprint`
- `scan_topic: /scan`
- `resolution: 0.05`
- `minimum_travel_distance: 0.10`
- `minimum_travel_heading: 0.10`
- `max_laser_range: 12.0`
- `use_scan_matching: true`
- `do_loop_closing: true`

Đây là file gốc cần dùng trong dự án:

```text
ros2_ws/src/libot_bringup/config/mapper_params_online_async.yaml
```

---

## 6. Launch file mapping

File launch chính cần chạy:

```text
ros2_ws/src/libot_bringup/launch/mapping.launch.py
```

Nội dung launch cần khởi động:
- `robot_state_publisher`
- `libot_stm32` bridge
- lidar driver (`rplidar_ros` hoặc tương đương)
- `slam_toolbox`
- `rviz2`

### Launch command

```bash
ros2 launch libot_bringup mapping.launch.py stm32_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
```

Nếu cổng khác, thay bằng giá trị đúng:

```bash
ros2 launch libot_bringup mapping.launch.py stm32_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
```

---

## 7. Workflow build ROS

### 7.1 Build workspace

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select libot_description libot_stm32 libot_bringup
source install/setup.bash
```

### 7.2 Nếu chạy từ Docker

```bash
cd /path/to/libot_pi
docker compose -f docker/compose.yaml up -d
docker exec -it <container_name> bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select libot_description libot_stm32 libot_bringup
source install/setup.bash
```

---

## 8. Workflow điều khiển robot

### 8.1 Chạy teleop bằng bàn phím

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Bản đồ chỉ tạo chính xác khi robot di chuyển đáng tin cậy, nên cần:
- đi chậm,
- giữ khoảng cách khỏi tường,
- không quay quá đột ngột,
- tránh vội vàng khi di chuyển trong không gian nhỏ.

### 8.2 Kiểm tra topic odom và scan

```bash
ros2 topic echo /odom
ros2 topic echo /scan
```

Nếu `/odom` không có dữ liệu hoặc sai, SLAM sẽ rất khó map đúng.

---

## 9. Workflow lưu map

Sau khi hoàn thành phần vẽ map:
#lưu map
```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/libot_bringup/maps/libot_map 
```

Lệnh này sẽ sinh ra:

- `/ros2_ws/src/libot_bringup/maps/libot_map.yaml`
- `/ros2_ws/src/libot_bringup/maps/libot_map.pgm`

Kiểm tra file bằng:

```bash
ls -l /ros2_ws/src/libot_bringup/maps
```

---

## 10. Workflow kiểm tra STM32 + ROS

### 10.1 Kiểm tra bridge

```bash
ros2 topic list | grep -E "cmd_vel|odom|scan"
```

### 10.2 Điều khiển bằng topic

```bash
echo "Thử publish cmd_vel nếu cần" 
```

Ví dụ:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

### 10.3 Nếu firmware STM32 đang đọc ký tự

Bridge ROS hiện tại cần khớp với protocol firmware:
- `W`, `A`, `S`, `D`, `X`
- hoặc parse dạng `VEL,...` nếu firmware đã cập nhật

---

## 11. Vấn đề quan trọng với xe 4 bánh

Robot 4 bánh không làm thay đổi nguyên tắc mapping, nhưng cần đảm bảo:

1. `cmd_vel` được chuyển về bánh trái / bánh phải hợp lý
2. `odom` được publish chính xác
3. `base_link` và `laser` frame khớp `URDF`
4. `scan_topic` đúng là `/scan`
5. `base_frame` đúng là `base_footprint` hoặc `base_link` tùy robot

Nếu robot 4 bánh dùng kiểu skid steer, nên chuyển `linear.x` + `angular.z` thành tốc độ bánh trái/phải.

---

## 12. Checklist cuối cùng

### Trước khi chạy mapping

- [ ] Lidar hoạt động, topic `/scan` có dữ liệu
- [ ] URDF được publish đúng bởi `robot_state_publisher`
- [ ] `odom` có dữ liệu hoặc encoder/IMU hoạt động
- [ ] `stm32_bridge` đang chạy
- [ ] `slam_toolbox` đang chạy
- [ ] `rviz2` hiển thị map và laser scan

### Sau khi chạy mapping

- [ ] Robot di chuyển đều và không va chạm
- [ ] Map xuất hiện trong RViz
- [ ] Không có drift quá lớn
- [ ] Dùng `map_saver_cli` lưu map
- [ ] Kiểm tra file `.yaml` và `.pgm`

---

## 13. Kết luận

Để triển khai mapping cho LiBot, cần tập trung vào 3 phần chính:

1. `Lidar + SLAM`
2. `STM32 + odometry + cmd_vel`
3. `Map save + lưu output`

Nếu các thành phần này đồng bộ tốt, robot sẽ tự động vẽ bản đồ 2D trong không gian nội thất.

---

## 14. Tài liệu tham khảo nhanh

- `ros2_ws/src/libot_bringup/launch/mapping.launch.py`
- `ros2_ws/src/libot_bringup/config/mapper_params_online_async.yaml`
- `ros2_ws/src/libot_stm32/libot_stm32/stm32_bridge.py`
- `ros2_ws/src/libot_description/urdf/libot.urdf.xacro`

---

## 15. Lệnh khởi chạy nhanh

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
source install/setup.bash
ros2 launch libot_bringup mapping.launch.py
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/libot_bringup/maps/libot_map
```
