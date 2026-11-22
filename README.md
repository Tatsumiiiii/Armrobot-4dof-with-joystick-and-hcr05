# Armrobot-4dof-with-joystick-and-hcr05

Dự án cánh tay robot 4 bậc dùng Arduino Uno, có thể điều khiển thủ công bằng hai cần Joystick và tự động gắp vật ở ba vị trí nhờ cảm biến siêu âm, kèm hiển thị góc các khớp theo thời gian thực qua Serial Monitor.

## Tổng quan

Hệ thống được thiết kế theo kiến trúc phân tầng, bao gồm:

- **Tầng đầu vào (Input Layer):** hai cần Joystick PS2 (điều khiển thủ công) và cảm biến siêu âm HC-SR05 (phát hiện vật, đo khoảng cách).
- **Tầng xử lý (Control Layer):** Arduino Uno chạy cơ chế góc mục tiêu (`target`, `angle`) để servo chuyển động mượt, cùng máy trạng thái và kịch bản từng bước cho chế độ tự động.
- **Tầng chấp hành (Actuator Layer):** bốn servo SG90 điều khiển xoay đế, vai, khuỷu và kẹp, cấp nguồn ngoài 5V.
- **Tầng giám sát (Monitoring Layer):** Serial Monitor (9600 baud) hiển thị góc từng khớp, khoảng cách đo và các bước của kịch bản.

## Ảnh và video demo

- Ảnh mô hình:
![alt text](<Poster báo cáo hệ thống nhúng (1).png>)

## Chức năng chính

- Điều khiển thủ công bốn khớp bằng hai cần Joystick, gạt càng mạnh servo quay càng nhanh, thả cần thì giữ nguyên vị trí
- Chuyển động mượt nhờ cơ chế góc mục tiêu, servo tiến từng bước nhỏ thay vì nhảy thẳng
- Hiển thị góc các khớp trên Serial Monitor để đo và hiệu chỉnh tư thế
- Phát hiện vật ở 3 ô bằng cảm biến siêu âm, đo 5 lần rồi lấy giá trị trung vị để giảm nhiễu
- Tự động gắp vật, mang tới vị trí đích, thả vật và quay về tư thế mở đầu, các servo di chuyển lần lượt từng cái
- Cơ chế an toàn: giới hạn góc từng khớp (`minA`, `maxA`), tạm dừng chế độ tự động khi người dùng gạt Joystick
- Thiết kế mô-đun, dễ thay đổi tư thế, thứ tự servo và các thông số thời gian

## Công nghệ sử dụng

- Arduino Uno
- Servo SG90 (4 chiếc)
- Shield Joystick meArm (2 cần PS2)
- Cảm biến siêu âm HC-SR05
- Thư viện `Servo`, giao tiếp Serial 9600 baud
- Khung cánh tay mica 4 bậc, nguồn ngoài 5V

## Sơ đồ chân

| Thiết bị | Chân thiết bị | Chân Arduino |
|---|---|---|
| Servo đế | Tín hiệu | D11 |
| Servo vai | Tín hiệu | D10 |
| Servo khuỷu | Tín hiệu | D9 |
| Servo kẹp | Tín hiệu | D6 |
| Joystick trái | X, Y | A0, A1 |
| Joystick phải | X, Y | A2, A3 |
| HC-SR05 | TRIG, ECHO | D12, D13 |
| HC-SR05 | VCC, GND | 5V, GND |

Chân OUT của HC-SR05 (nếu có) để trống.

## Chế độ hoạt động

### Chế độ thủ công

Gạt Joystick để điều khiển từng khớp. Mỗi lần gạt, góc hiện tại của 4 servo được in ra Serial Monitor.

### Chế độ tự động

Khi vật nằm trong một trong ba ô, cánh tay thực hiện lần lượt:

1. Mở kẹp
2. Vươn tới ô: đế, rồi vai, rồi khuỷu
3. Khép kẹp để gắp
4. Mang tới vị trí đích
5. Mở kẹp thả vật
6. Về tư thế mở đầu
7. Chờ các ô trống rồi mới nhận vật tiếp theo

Khi gạt Joystick giữa chừng, chế độ tự động tạm dừng 3 giây, sau đó cánh tay tự về tư thế mở đầu.

### Ba ô phát hiện (tính từ mặt cảm biến)

| Ô | Khoảng cách | Tư thế `{đế, vai, khuỷu, kẹp}` |
|---|---|---|
| Ô 1 | 8 đến 10 cm | `{90, 80, 92, 90}` |
| Ô 2 | 11 đến 13 cm | `{90, 108, 106, 90}` |
| Ô 3 | 13 đến 15 cm | `{90, 137, 114, 84}` |
| Mở đầu | | `{90, 122, 61, 90}` |
| Đích | | `{45, 100, 74, 90}` |

Các số này được đo bằng thực nghiệm trên mô hình của nhóm, cần hiệu chỉnh lại nếu thay đổi khung, vị trí cảm biến hoặc mặt bàn.


## Cài đặt và chạy

1. Cài Arduino IDE. Thư viện `Servo` có sẵn, không cần cài thêm.
2. Mở file chương trình bằng Arduino IDE (đổi đuôi `.cpp` thành `.ino` và đặt thư mục trùng tên với file), hoặc dùng PlatformIO.
3. Chọn bo mạch **Arduino Uno** và đúng cổng COM, rồi nạp chương trình.
4. Mở Serial Monitor, chọn **9600 baud** để xem góc và dữ liệu đo.
5. Cấp nguồn ngoài 5V đủ dòng cho các servo.

## Hiệu chỉnh

1. Nạp `do_goc_servo.cpp`, dùng Joystick đẩy chậm từng khớp để dò giới hạn góc, cập nhật `minA`, `maxA` (lùi lại khoảng 5° để dự phòng).
2. Đặt vật vào từng ô, đọc khoảng cách trên Serial Monitor để chọn ranh giới `ZONE_LO`, `ZONE_HI`.
3. Dùng Joystick đưa ngàm kẹp tới từng ô, tới đích và tư thế mở đầu, ghi bộ bốn góc hiển thị vào `ZONE_POSE`, `DEST_POSE`, `START_POSE`.
4. Ở tư thế mở đầu, không đặt vật nào, kiểm tra Serial Monitor không báo có vật.

## Thông số chính

| Thông số | Giá trị | Ý nghĩa |
|---|---|---|
| `SERVO_INTERVAL` | 40 ms | Nhịp cập nhật servo, càng lớn càng chậm |
| `SMOOTH_STEP` | 2° | Bước tối đa mỗi nhịp của servo |
| `DEADZONE` | 80 | Vùng chết quanh tâm Joystick |
| `SENSE_INTERVAL` | 60 ms | Khoảng cách giữa hai lần đo siêu âm |
| `NUM_SAMPLES`, `MIN_AGREE` | 5, 3 | Đo 5 lần, ít nhất 3 lần cùng ô mới chấp nhận |
| `MANUAL_LOCK_MS` | 3000 ms | Tạm dừng tự động sau khi gạt Joystick |
| `minA` / `maxA` | `{0, 30, 20, 25}` / `{180, 150, 160, 95}` | Giới hạn góc an toàn |

## Lưu ý an toàn

- Dùng nguồn ngoài 5V đủ dòng cho bốn servo và thêm tụ 470 đến 1000 µF gần hàng servo. Cổng USB chỉ cấp khoảng 500 mA, không đủ khi servo chạy cùng lúc.
- Tất cả thiết bị phải nối chung GND.
- Không cắm USB thông thường cùng lúc với nguồn ngoài đang cấp vào chân 5V, vì hai nguồn 5V sẽ nối song song.
- Dò kỹ giới hạn góc trước khi chạy tự động, tránh để servo bị chặn cơ khí.

## Hướng phát triển

- Thay servo bánh răng kim loại (MG90S, MG996R) và gia cố khung
- Thêm cảm biến xác nhận gắp thành công
- Dùng cảm biến khoảng cách ToF hoặc camera để nhận diện vật linh hoạt hơn
- Ghi và phát lại động tác, lưu bằng EEPROM
- Điều khiển không dây bằng Bluetooth hoặc WiFi (ESP32, MQTT)
- Ứng dụng động học nghịch để vươn tới tọa độ tùy ý

## 🤝 Đóng góp

Dự án được phát triển bởi các thành viên:

| Họ và Tên | Vai trò |
|---|---|
| Hồ Khắc Sơn | Cập nhật sau. |
| Bùi Anh Tuấn | Cập nhật sau. |
| Nguyễn Hữu Mạnh | Cập nhật sau. |
