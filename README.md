# Armrobot-4dof-with-joystick-and-hcr05

Dự án cánh tay robot 4 bậc dùng Arduino Uno/Nano, có thể điều khiển thủ công bằng hai cần Joystick và tự động gắp vật ở ba vị trí nhờ cảm biến siêu âm, kèm hiển thị góc các khớp và dữ liệu đo theo thời gian thực qua Serial Monitor. Servo được điều khiển qua module PCA9685 (16 kênh PWM, giao tiếp I2C).

## Tổng quan

Hệ thống được thiết kế theo kiến trúc phân tầng, bao gồm:

- **Tầng đầu vào (Input Layer):** hai cần Joystick PS2 (điều khiển thủ công) và cảm biến siêu âm HC-SR05 (phát hiện vật, đo khoảng cách).
- **Tầng xử lý (Control Layer):** Arduino Uno/Nano chạy cơ chế góc mục tiêu (`target`, `angle`) để servo chuyển động mượt, cùng máy trạng thái (`S_IDLE`, `S_RUN`, `S_WAIT_CLEAR`) và kịch bản từng bước cho chế độ tự động.
- **Tầng chấp hành (Actuator Layer):** module PCA9685 (I2C) xuất xung PWM cho bốn servo điều khiển xoay đế, vai, khuỷu và kẹp. Servo được cấp nguồn riêng 5V từ UBEC.
- **Tầng giám sát (Monitoring Layer):** Serial Monitor (9600 baud) hiển thị góc từng khớp, kết quả đo khoảng cách, trạng thái chờ ô trống và các bước của kịch bản.

## Ảnh và video demo

- Ảnh mô hình:
![alt text](<PPoster_CNTT5_Aiot.png>)

## Chức năng chính

- Điều khiển thủ công bốn khớp bằng hai cần Joystick, gạt càng mạnh servo quay càng nhanh, thả cần thì giữ nguyên vị trí
- Chuyển động mượt nhờ cơ chế góc mục tiêu, servo tiến từng bước nhỏ thay vì nhảy thẳng
- Hiển thị góc các khớp trên Serial Monitor để đo và hiệu chỉnh tư thế
- Phát hiện vật ở 3 ô bằng cảm biến siêu âm, đo 5 lần rồi lấy giá trị trung vị, ít nhất 3/5 lần đo phải rơi vào cùng một ô thì mới chấp nhận
- Tự động gắp vật, mang tới vị trí đích, thả vật và quay về tư thế mở đầu, các servo di chuyển lần lượt từng cái
- Chờ ô trống có dung sai nhiễu và có giới hạn thời gian, tránh bị kẹt khi cảm biến đo nhiễu
- Cơ chế an toàn: giới hạn góc từng khớp (`minA`, `maxA`), tạm dừng chế độ tự động khi người dùng gạt Joystick
- Thiết kế mô-đun, dễ thay đổi tư thế, thứ tự servo và các thông số thời gian

## Công nghệ sử dụng

- Arduino Uno/Nano
- Module PWM PCA9685 (16 kênh, I2C, địa chỉ mặc định `0x40`)
- Servo SG90 (4 chiếc)
- Shield Joystick meArm (2 cần PS2)
- Cảm biến siêu âm HC-SR05
- Mạch hạ áp UBEC 5V cấp nguồn cho servo
- Thư viện `Adafruit PWM Servo Driver Library`, `Wire`, giao tiếp Serial 9600 baud
- Khung cánh tay mica 4 bậc

## Sơ đồ chân

| Thiết bị | Chân thiết bị | Chân Arduino / nguồn |
|---|---|---|
| PCA9685 | VCC (nguồn logic) | 5V Arduino |
| PCA9685 | GND | GND (nối chung với UBEC) |
| PCA9685 | SDA | A4 |
| PCA9685 | SCL | A5 |
| PCA9685 | V+ (cọc vặn vít, nguồn servo) | 5V từ UBEC |
| Servo đế | Tín hiệu | Kênh 0 của PCA9685 |
| Servo vai | Tín hiệu | Kênh 1 của PCA9685 |
| Servo khuỷu | Tín hiệu | Kênh 2 của PCA9685 |
| Servo kẹp | Tín hiệu | Kênh 3 của PCA9685 |
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
4. Mang tới vị trí đích: khuỷu, rồi vai, rồi đế
5. Mở kẹp thả vật
6. Về tư thế mở đầu: đế, rồi vai, rồi khuỷu, rồi kẹp
7. Chờ các ô trống rồi mới nhận vật tiếp theo

Ở bước 7, chương trình đo mỗi 160 ms và cộng 1 điểm khi ô trống. Khi đo thấy có vật (do nhiễu hoặc vật còn sót lại) thì chỉ trừ 1 điểm, không về 0. Đủ 5 điểm thì quay lại nhận vật. Nếu sau 8 giây vẫn chưa đủ điểm, chương trình in thông báo `Het gio cho o trong, tiep tuc do` rồi tự quay lại đo. Nếu lúc đó vật vẫn còn trong một ô, tay sẽ gắp lại vật đó.

Khi gạt Joystick giữa chừng, chế độ tự động tạm dừng 3 giây, sau đó cánh tay tự về tư thế mở đầu.

### Ba ô phát hiện (tính từ mặt cảm biến)

| Ô | Khoảng cách | Tư thế `{đế, vai, khuỷu, kẹp}` |
|---|---|---|
| Ô 1 | 79 đến dưới 100 mm | `{90, 82, 92, 90}` |
| Ô 2 | 100 đến dưới 119 mm | `{90, 103, 101, 90}` |
| Ô 3 | 119 đến dưới 140 mm | `{90, 120, 114, 84}` |
| Mở đầu | | `{90, 122, 61, 90}` |
| Đích | | `{38, 100, 74, 90}` |

Ba ô nối liền nhau, vật dưới 79 mm hoặc từ 140 mm trở lên bị bỏ qua. Số cuối trong tư thế của mỗi ô là góc kẹp khi khép để gắp vật (kẹp mở là 15°).

Các số này được đo bằng thực nghiệm trên mô hình của nhóm, cần hiệu chỉnh lại nếu thay đổi khung, vị trí cảm biến hoặc mặt bàn.

## Cài đặt và chạy

1. Cài Arduino IDE.
2. Vào Library Manager, cài thư viện **Adafruit PWM Servo Driver Library** (kèm các thư viện phụ thuộc nếu IDE hỏi). Thư viện `Wire` có sẵn.
3. Mở file chương trình bằng Arduino IDE (đổi đuôi `.cpp` thành `.ino` và đặt thư mục trùng tên với file), hoặc dùng PlatformIO.
4. Chọn bo mạch **Arduino Uno** (hoặc Nano) và đúng cổng COM, rồi nạp chương trình.
5. Mở Serial Monitor, chọn **9600 baud** để xem góc và dữ liệu đo.
6. Cấp nguồn 5V đủ dòng từ UBEC vào chân V+ của PCA9685 để chạy servo.

## Hiệu chỉnh

1. Nạp `do_goc_servo.cpp`, dùng Joystick đẩy chậm từng khớp để dò giới hạn góc, cập nhật `minA`, `maxA` (lùi lại khoảng 5° để dự phòng).
2. Đặt vật vào từng ô, đọc khoảng cách trên Serial Monitor để chọn ranh giới `ZONE_LO`, `ZONE_HI` (đơn vị mm).
3. Dùng Joystick đưa ngàm kẹp tới từng ô, tới đích và tư thế mở đầu, ghi bộ bốn góc hiển thị vào `ZONE_POSE`, `DEST_POSE`, `START_POSE`.
4. Ở tư thế mở đầu, không đặt vật nào, kiểm tra Serial Monitor không báo có vật.
5. Nếu góc servo bị lệch so với mong muốn, kiểm tra `SERVO_MIN_US`, `SERVO_MAX_US` và `OSC_FREQ` (dao động nội của PCA9685 có thể lệch so với 27 MHz).

## Thông số chính

| Thông số | Giá trị | Ý nghĩa |
|---|---|---|
| `SERVO_INTERVAL` | 40 ms | Nhịp cập nhật servo, càng lớn càng chậm |
| `SMOOTH_STEP` | 2° | Bước tối đa mỗi nhịp của servo |
| `DEADZONE` | 80 | Vùng chết quanh tâm Joystick |
| `PWM_FREQ` | 50 Hz | Tần số xung điều khiển servo |
| `SERVO_MIN_US`, `SERVO_MAX_US` | 544, 2400 µs | Độ rộng xung ứng với 0° và 180° |
| `SENSE_INTERVAL` | 160 ms | Khoảng cách giữa hai lần đo siêu âm |
| `NUM_SAMPLES`, `MIN_AGREE` | 5, 3 | Đo 5 lần, ít nhất 3 lần cùng ô mới chấp nhận |
| `CLEAR_N` | 5 | Số điểm "trống" cần đạt để nhận vật mới |
| `CLEAR_TIMEOUT_MS` | 8000 ms | Chờ ô trống tối đa, hết giờ thì quay lại đo |
| `ECHO_TIMEOUT_US` | 4000 µs | Thời gian nghe tiếng dội tối đa (khoảng 65 cm) |
| `MANUAL_LOCK_MS` | 3000 ms | Tạm dừng tự động sau khi gạt Joystick |
| `minA` / `maxA` | `{0, 30, 20, 15}` / `{180, 150, 160, 95}` | Giới hạn góc an toàn |

## Xử lý sự cố

| Hiện tượng trên Serial Monitor | Nguyên nhân thường gặp |
|---|---|
| In liên tục `Cho trong: ... mm` với số nằm trong 79 đến 139 | Có vật còn sót lại hoặc thân tay, mặt bàn dội tiếng vào cảm biến |
| In liên tục `Goc: ...` khi không chạm cần | Joystick lệch tâm hoặc nhiễu nguồn, tăng `DEADZONE` |
| Không in gì và servo không phản ứng | Sụt nguồn hoặc nhiễu I2C, kiểm tra tụ lọc và dây SDA/SCL |

## Lưu ý an toàn

- Cấp nguồn servo bằng UBEC 5V đủ dòng cho bốn servo và thêm tụ 470 đến 1000 µF gần chân V+ của PCA9685. Cổng USB chỉ cấp khoảng 500 mA, không đủ khi servo chạy cùng lúc.
- Tất cả thiết bị (Arduino, PCA9685, UBEC, cảm biến) phải nối chung GND.
- Không cắm USB thông thường cùng lúc với nguồn ngoài đang cấp vào chân 5V của Arduino, vì hai nguồn 5V sẽ nối song song.
- Không cấp nguồn servo qua chân VCC của PCA9685, chân này chỉ dành cho mạch logic. Nguồn servo đi vào chân V+.
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
