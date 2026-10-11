# Giải thích source code đồ án LiBot

*Tổng quan kiến trúc • Luồng vận hành • Điểm cần lưu ý • Câu hỏi bảo vệ*

## 1. Tổng quan dự án

Repo hiện có hai luồng code chính:

1. **Ứng dụng Python** tại `robot_app`: tổ chức và thực thi các bước workflow.
2. **Workspace ROS 2** tại `ros2_ws/src`: kết nối LiDAR, STM32, odometry, TF và SLAM.

Cần phân biệt phần đã triển khai với phần mới là khung mẫu hoặc mô tả trong tài liệu.

### Ứng dụng Python

Điểm bắt đầu là `robot_app/main.py`. Chương trình đọc `robot_app/config/settings.json`, khởi tạo `TCPServer` và `WorkflowEngine`, nạp workflow cấu hình sẵn, chạy TCP server trên thread nền rồi thực thi danh sách step.

`WorkflowEngine` trong `robot_app/core/workflow_engine.py` ánh xạ tên loại step sang lớp tương ứng và chạy tuần tự. `RobotContext` trong `robot_app/core/context.py` giữ `variables`, `step_results` và `metadata` để trao đổi dữ liệu giữa các bước.

`TCPServer` trong `robot_app/core/tcp_server.py` mặc định lắng nghe `0.0.0.0:9000`. Hiện nó nhận chuỗi, in nội dung và phản hồi `ACK`; chưa phân tích JSON thành workflow hay điều khiển robot. `settings.json` đang có `workflow.steps` rỗng nên ứng dụng hiện chạy không có step nào.

### ROS 2, STM32 và SLAM

Launch mapping chính là `ros2_ws/src/libot_bringup/launch/mapping.launch.py`. Nó khởi chạy mô tả robot, driver RPLIDAR, node `libbot_stm32/stm32_bridge` và SLAM Toolbox. Mặc định `lidar_port` là `/dev/rplidar`, `stm32_port` là `/dev/serial0`.

Luồng dữ liệu mapping: LiDAR phát `/scan`; STM32 gửi encoder qua UART; `stm32_bridge` tính và phát `/odom` cùng TF `odom → base_footprint`; SLAM Toolbox kết hợp scan và odometry, tạo map và transform `map → odom`.

```text
LiDAR → /scan ─────────────────────┐
                                   ├→ SLAM Toolbox → /map, TF map→odom
STM32 ↔ stm32_bridge → /odom ──────┘
       ↑ /cmd_vel
```

## 2. Các file cần hiểu

- `robot_app/main.py` — điểm vào ứng dụng Python, đọc cấu hình và khởi chạy server/workflow.
- `robot_app/config/settings.json` — cấu hình TCP, thiết bị và danh sách workflow; danh sách steps hiện rỗng.
- `robot_app/core/workflow_engine.py` — đăng ký loại step và chạy step tuần tự.
- `robot_app/core/context.py` — vùng dữ liệu dùng chung giữa các step.
- `robot_app/core/tcp_server.py` — socket TCP; hiện mới nhận chuỗi và trả `ACK`.
- `robot_app/steps/base_step.py` — giao diện `execute(context)` chung cho các bước.
- `robot_app/steps/move.py` — giải quyết đích từ cấu hình/context, tạo kết quả MOVE và cập nhật trạng thái.
- `robot_app/steps/scan.py`, `process_data.py`, `evaluate.py` — bước quét, xử lý dữ liệu, đánh giá điều kiện; hiện chủ yếu là logic mẫu.
- `robot_app/steps/display.py`, `wait.py`, `notify.py` — cập nhật thông báo trong context hoặc chờ; chưa kết nối UI/loa thật.
- `robot_app/hardware/navigation.py`, `barcode_scanner.py`, `speaker.py`, `ui_controller.py` — lớp trừu tượng phần cứng dạng stub.
- `ros2_ws/src/libot_bringup/launch/mapping.launch.py` — launch mapping đang có; khởi động các thành phần ROS.
- `ros2_ws/src/libot_bringup/config/mapper_params_online_async.yaml` — frame, scan topic, độ phân giải, scan matching và loop closure.
- `ros2_ws/src/libot_bringup/config/slam.rviz` — bố trí hiển thị `/map`, `/scan`, RobotModel và TF.
- `ros2_ws/src/libbot_stm32/libbot_stm32/stm32_bridge.py` — nhận `/cmd_vel`, giao tiếp UART, xử lý encoder/pin, phát odometry và TF.
- `ros2_ws/src/libot_description/urdf/libot.urdf.xacro` và `launch/description.launch.py` — mô tả frame robot và chạy `robot_state_publisher`.
- `ros2_ws/src/libot_bringup/scripts/manual_control.py` — điều khiển tay bằng bàn phím, phát `Twist` lên `/cmd_vel`.
- `docker/Dockerfile`, `docker/compose.yaml`, `docker/entrypoint.sh` — môi trường ROS 2 Jazzy, thiết bị serial, mount thư mục và entrypoint.
- `tests/test_move_step.py` — kiểm tra logic MOVE; không chứng minh xe di chuyển thật.

## 3. Các giới hạn cần trình bày trung thực

- **MOVE chưa tích hợp Nav2:** `NavigationController.move_to()` chỉ trả lại tọa độ. MOVE với endpoint ID còn có thể trả `completed` mà không gửi goal đến robot; trạng thái hoàn thành hiện không phải xác nhận chuyển động vật lý.
- **SCAN chưa đọc camera/barcode:** `ScanStep` trả `no_data`; lớp `BarcodeScanner` trả trạng thái `idle`.
- **PROCESS_DATA** chỉ lưu gói dữ liệu dựa trên `last_scan`; chưa có đối chiếu sách hoặc gọi API.
- **EVALUATE** chỉ chuyển `condition` cấu hình thành boolean. `WorkflowEngine` chưa dùng kết quả đó để phân nhánh hoặc bỏ qua step.
- **DISPLAY và NOTIFY** mới cập nhật context, chưa gọi màn hình hoặc loa.
- Workflow chạy tuần tự và tiếp tục qua các step sau, kể cả khi step trước báo lỗi; chưa có chính sách retry, hủy hay dừng an toàn.
- Các file `navigation.launch.py`, `robot.launch.py`, `config/nav2.yaml`, `slam.yaml` và `ekf.yaml` đang rỗng. Package `libot_barcode` cũng có `package.xml`, `setup.py` và `barcode_node.py` rỗng.
- Docker Compose chạy lệnh `tail -f /dev/null`; nó giữ container sống nhưng không tự chạy ứng dụng Python hay mapping launch.
- `MAPPING_WORKFLOW.md` có hướng dẫn Humble và tên `libot_stm32`, trong khi Docker/`run.md` dùng Jazzy và package thực tế là `libbot_stm32`. Nên xác nhận theo cấu hình đang chạy.
- URDF ghi vị trí LiDAR là tạm. Thông số bánh, CPR encoder, dấu encoder và mô hình mecanum trong bridge cần kiểm chứng/hiệu chuẩn thực tế.
- `manual_control.py` đặt tốc độ tuyến tính `1 m/s` và góc `2 rad/s`, khá cao để thử nghiệm; thông báo reset speed trong log không khớp giá trị thực tế đặt lại.
- Compose mount map tại `/ros2_ws/maps`, trong khi hướng dẫn lưu map có ví dụ ở đường dẫn khác. Cần thống nhất vị trí lưu để map được giữ lại.

## 4. Cách kiểm tra khi chạy mapping

1. Build workspace theo hướng dẫn trong `run.md`, source `/opt/ros/jazzy/setup.bash` và `install/setup.bash`.
2. Chạy `mapping.launch.py` với cổng thiết bị đúng của robot.
3. Kiểm tra `/scan` và `/odom` có dữ liệu; xác nhận cây TF:

   ```text
   map → odom → base_footprint → base_link → laser
   ```

4. Mở RViz với `Fixed Frame` là `map`; quan sát map, scan, robot model và TF.
5. Tránh khởi chạy trùng `robot_state_publisher`, driver LiDAR, SLAM hay publisher TF tĩnh; chạy trùng có thể gây TF sai hoặc map lệch.
6. Lưu map bằng `nav2_map_server map_saver_cli` và kiểm tra cả file YAML lẫn ảnh PGM.

## 5. Câu hỏi có thể gặp khi bảo vệ

### 1. Đồ án gồm những thành phần nào? Raspberry Pi và STM32 có vai trò gì?

Trình bày vai trò dự kiến của ứng dụng workflow và ROS 2 trên Pi, STM32 điều khiển động cơ/đọc encoder, cùng backend. Nói rõ thành phần nào trong repo đã kết nối thật.

### 2. Luồng lệnh điều khiển từ ROS đến động cơ đi thế nào?

Theo code ROS: `/cmd_vel` → callback `stm32_bridge` → gói UART `VEL,...` → firmware STM32. TCP server Python hiện chưa biến thông điệp backend thành goal điều hướng.

### 3. Robot tạo bản đồ bằng cách nào?

LiDAR cung cấp `/scan`; encoder cung cấp chuyển động và `/odom`; SLAM Toolbox kết hợp hai nguồn để ước lượng map và vị trí robot.

### 4. TF là gì và vì sao chuỗi TF phải đúng?

TF mô tả quan hệ tọa độ giữa các frame. Sai frame hoặc nhiều node phát cùng transform có thể làm scan/robot lệch và bản đồ không chính xác.

### 5. Odometry encoder được tính thế nào?

Chuyển chênh lệch encoder thành quãng đường qua chu vi bánh và CPR; áp dụng hình học/dấu bánh theo mô hình cơ cấu; tích phân chuyển động thành pose. Cần hiệu chuẩn thông số thực.

### 6. Những tham số SLAM nào cần chú ý?

`map_frame`, `odom_frame`, `base_frame`, `scan_topic`, `resolution`, ngưỡng di chuyển, `max_laser_range`, scan matching và loop closure.

### 7. Workflow Engine mở rộng bằng cách nào?

Mỗi step cùng giao diện `execute(context)`, được đăng ký trong `STEP_TYPES`; `RobotContext` chia sẻ dữ liệu. Hiện engine chạy tuần tự, chưa có nhánh điều kiện thực sự.

### 8. Backend gửi JSON workflow thì robot làm gì?

Theo code hiện có, TCP server chỉ in chuỗi nhận được và trả `ACK`; chưa parse JSON hay nạp workflow động.

### 9. Làm sao xác nhận MOVE thành công?

Trong phiên bản hiện tại, kết quả chủ yếu là logic/stub và chưa xác nhận Nav2 action result hoặc pose thực. Tích hợp thật cần theo dõi kết quả action, timeout, hủy và lỗi.

### 10. Hệ thống xử lý mất encoder hoặc mất `/cmd_vel` thế nào?

Bridge có timeout encoder để đưa vận tốc odometry về 0 và watchdog gửi lệnh vận tốc 0 khi mất lệnh. An toàn cuối cùng vẫn cần watchdog firmware và thử nghiệm mất UART/ROS.

### 11. Đã kiểm thử những gì?

`test_move_step.py` kiểm tra logic MOVE với destination và trường hợp thiếu đích. Đây là unit test, không thay thế kiểm thử tích hợp ROS hay chạy robot thật.

### 12. Hạn chế và hướng phát triển tiếp theo?

Nối TCP với workflow JSON; tích hợp MOVE với Nav2; hoàn thiện barcode, UI và speaker; bổ sung navigation; hiệu chuẩn odometry/URDF; thêm kiểm thử tích hợp và cơ chế an toàn.

## 6. Câu chốt nên nhớ

Đồ án đã có nền tảng mapping ROS 2 và khung workflow Python. Tuy nhiên, luồng backend → workflow động và một số tích hợp navigation, barcode, UI/loa vẫn chưa hoàn chỉnh trong source hiện tại. Khi bảo vệ, nên mô tả đúng phạm vi đã làm và xem các tích hợp còn thiếu là hướng phát triển.

## 7. Tài liệu tham khảo trong repo

- `README.md` — mô tả tổng quan, kiến trúc dự kiến và các step.
- `run.md` — các lệnh Docker/ROS 2 mapping theo Jazzy.
- `MAPPING_WORKFLOW.md` — ghi chú quy trình mapping; cần đối chiếu phiên bản ROS và tên package.
