/*
  Cánh tay robot 4 bậc - tự động phát hiện vật ở 3 ô, gắp, mang tới ĐÍCH rồi về vị trí mở đầu.
  Các servo di chuyển LẦN LƯỢT TỪNG CÁI (đế -> vai -> khuỷu), không chạy cùng lúc.
  Joystick vẫn dùng được để chỉnh tay (khi gạt, chế độ tự động tạm dừng).

  Servo    : D11 (đế), D10 (vai), D9 (khuỷu), D6 (kẹp)
  Joystick : A0, A1 (cần trái X, Y), A2, A3 (cần phải X, Y)
  HC-SR05  : VCC = 5V, GND = GND, TRIG = D12, ECHO = D13 (chân OUT để trống)

  Một lượt gắp (khi thấy vật trong ô N):
    1. Mở kẹp
    2. Vươn tới tư thế ô N:  đế -> vai -> khuỷu
    3. Khép kẹp để gắp (góc kẹp lấy từ tư thế của ô N)
    4. Mang tới ĐÍCH:        đế -> vai -> khuỷu
    5. Mở kẹp thả vật
    6. Về vị trí MỞ ĐẦU:     đế -> vai -> khuỷu -> kẹp
    7. Chờ các ô trống rồi mới nhận vật tiếp theo
*/

#include <Arduino.h>
#include <Servo.h>

// ---------- Chân ----------
const byte TRIG_PIN = 12;
const byte ECHO_PIN = 13;
const byte servoPins[4] = {11, 10, 9, 6};     // đế, vai, khuỷu, kẹp
const byte joyPins[4]   = {A0, A1, A2, A3};

// ---------- Giới hạn servo ----------
const int minA[4] = {0, 30, 20, 25};
const int maxA[4] = {180, 150, 160, 95};
int dir[4] = {1, 1, 1, 1};                    // đổi thành -1 nếu gạt cần ngược chiều mong muốn

// ---------- Các tư thế: {đế, vai, khuỷu, kẹp} ----------
const int START_POSE[4] = {90, 122, 61, 90};  // mở đầu
const int DEST_POSE[4]  = {45, 100, 74, 90};  // đích (nơi thả vật)

// ---------- 3 ô: khoảng cách (mm) tính từ MẶT CẢM BIẾN ----------
const byte NUM_ZONES = 3;
const int ZONE_LO[NUM_ZONES] = {  80, 110, 130};   // mm (tính cả đầu này)
const int ZONE_HI[NUM_ZONES] = { 100, 130, 150};   // mm (không tính đầu này)
// Ô 1: 8-10 cm, ô 2: 11-13 cm, ô 3: 13-15 cm. Khoảng 10-11 cm, dưới 8 cm và trên 15 cm bị bỏ qua.

// Tư thế vươn tới từng ô. Số cuối là góc kẹp khi ĐÓNG GẮP vật.
const int ZONE_POSE[NUM_ZONES][4] = {
  {90,  80,  92, 90},    // ô 1
  {90, 108, 106, 90},    // ô 2
  {90, 137, 114, 84}     // ô 3
};

// ---------- Kẹp ----------
const int GRIP_OPEN = 40;               // góc kẹp mở

// ---------- Thứ tự servo khi di chuyển (0 = đế, 1 = vai, 2 = khuỷu) ----------
const byte ORDER_ZONE[3]  = {0, 1, 2};  // lúc vươn tới ô
const byte ORDER_DEST[3]  = {2 ,1 ,0};  // lúc mang tới đích
const byte ORDER_START[3] = {0, 1, 2};  // lúc về mở đầu

// ---------- Cài đặt phản hồi cảm biến ----------
const unsigned long SENSE_INTERVAL = 60;    // ms giữa 2 lần đo (chậm hơn để tiếng dội cũ tắt hẳn)
const byte NUM_SAMPLES = 5;                 // đo 5 lần rồi chọn giá trị ở giữa (trung vị)
const byte MIN_AGREE   = 3;                 // ít nhất 3/5 lần đo phải rơi vào cùng một ô
const int  CLEAR_N     = 5;                 // số lần đo liên tiếp "trống" để nhận vật mới
const unsigned long ECHO_TIMEOUT_US = 4000; // nghe tối đa ~65 cm

// ---------- Cài đặt thời gian và tốc độ ----------
const unsigned long STEP_PAUSE_MS   = 150;  // nghỉ giữa 2 servo liên tiếp
const unsigned long OPEN_SETTLE_MS  = 300;  // chờ kẹp mở trước khi vươn
const unsigned long REACH_SETTLE_MS = 600;  // dừng sau khi vươn tới, trước khi khép kẹp
const unsigned long GRAB_SETTLE_MS  = 1000; // chờ kẹp khép chắc
const unsigned long DROP_SETTLE_MS  = 800;  // chờ kẹp mở hết khi thả vật
const unsigned long MANUAL_LOCK_MS  = 3000; // tạm dừng tự động sau khi gạt joystick
const unsigned long SERVO_INTERVAL  = 40;   // ms mỗi nhịp cập nhật servo (càng lớn càng chậm)
const int DEADZONE     = 80;
const int JOY_MAX_STEP = 2;
const int SMOOTH_STEP  = 2;                 // servo tiến bao nhiêu độ mỗi nhịp
const bool DEBUG = true;

// ---------- Biến ----------
Servo servos[4];
int angle[4];
int target[4];
int center[4];

enum State { S_IDLE, S_RUN, S_WAIT_CLEAR };
State state = S_IDLE;

bool needStart = false;      // true: cần về vị trí mở đầu trước khi chờ vật (sau khi chỉnh tay)
bool pickRun   = false;      // kịch bản đang chạy là lượt gắp (true) hay chỉ về mở đầu (false)
int  samples[NUM_SAMPLES];   // các lần đo của đợt hiện tại
byte sampleCount = 0;
int  clearCount = 0;
unsigned long lastSense  = 0;
unsigned long lastServo  = 0;
unsigned long lastManual = 0;
unsigned long lastPrint  = 0;

// ---------- Kịch bản: danh sách bước, mỗi bước chỉ chạy MỘT servo ----------
struct Step {
  byte servo;
  int  angle;
  unsigned int pauseMs;     // nghỉ sau khi servo tới nơi
};
const byte MAX_STEPS = 20;
Step steps[MAX_STEPS];
byte stepCount = 0;
byte stepIdx = 0;
bool stepStarted = false;
bool stepArrived = false;
unsigned long stepDoneAt = 0;

void scriptClear() {
  stepCount = 0;
  stepIdx = 0;
  stepStarted = false;
  stepArrived = false;
}

void addStep(byte servo, int a, unsigned int pauseMs) {
  if (stepCount >= MAX_STEPS) return;
  steps[stepCount].servo = servo;
  steps[stepCount].angle = a;
  steps[stepCount].pauseMs = pauseMs;
  stepCount++;
}

// Thêm 3 bước (đế, vai, khuỷu) theo thứ tự order tới tư thế pose
void addPose(const int pose[4], const byte order[3]) {
  for (byte k = 0; k < 3; k++) addStep(order[k], pose[order[k]], STEP_PAUSE_MS);
}

void setLastPause(unsigned int ms) {
  if (stepCount > 0) steps[stepCount - 1].pauseMs = ms;
}

void buildPickScript(byte z) {
  scriptClear();
  addStep(3, GRIP_OPEN, OPEN_SETTLE_MS);          // 1. mở kẹp
  addPose(ZONE_POSE[z], ORDER_ZONE);              // 2. vươn tới ô
  setLastPause(REACH_SETTLE_MS);
  addStep(3, ZONE_POSE[z][3], GRAB_SETTLE_MS);    // 3. gắp
  addPose(DEST_POSE, ORDER_DEST);                 // 4. mang tới đích
  setLastPause(REACH_SETTLE_MS);
  addStep(3, GRIP_OPEN, DROP_SETTLE_MS);          // 5. thả vật
  addPose(START_POSE, ORDER_START);               // 6. về mở đầu
  addStep(3, START_POSE[3], STEP_PAUSE_MS);
  pickRun = true;
}

void buildStartScript() {
  scriptClear();
  addPose(START_POSE, ORDER_START);
  addStep(3, START_POSE[3], STEP_PAUSE_MS);
  pickRun = false;
}

void moveTo(byte i, int a) {
  target[i] = constrain(a, minA[i], maxA[i]);
}

// Chạy kịch bản từng bước một. Trả về true khi đã xong toàn bộ.
bool runScript() {
  if (stepIdx >= stepCount) return true;
  Step &s = steps[stepIdx];

  if (!stepStarted) {                              // bắt đầu bước: ra lệnh cho đúng 1 servo
    moveTo(s.servo, s.angle);
    stepStarted = true;
    stepArrived = false;
    if (DEBUG) {
      Serial.print(F("Buoc "));  Serial.print(stepIdx + 1);
      Serial.print('/');         Serial.print(stepCount);
      Serial.print(F(": servo ")); Serial.print(s.servo + 1);
      Serial.print(F(" -> "));   Serial.println(target[s.servo]);
    }
    return false;
  }
  if (!stepArrived) {                              // chờ servo tới nơi
    if (angle[s.servo] == target[s.servo]) {
      stepArrived = true;
      stepDoneAt = millis();
    }
    return false;
  }
  if (millis() - stepDoneAt >= s.pauseMs) {        // nghỉ xong, sang bước kế
    stepIdx++;
    stepStarted = false;
  }
  return stepIdx >= stepCount;
}

// ---------- Servo ----------
void updateServos() {
  for (byte i = 0; i < 4; i++) {
    if (angle[i] != target[i]) {
      int diff = target[i] - angle[i];
      angle[i] += constrain(diff, -SMOOTH_STEP, SMOOTH_STEP);
      servos[i].write(angle[i]);
    }
  }
}

// ---------- Joystick (trả về true nếu đang gạt) ----------
bool handleJoystick() {
  bool moved = false;
  for (byte i = 0; i < 4; i++) {
    int v = analogRead(joyPins[i]) - center[i];
    if (abs(v) > DEADZONE) {
      int step = map(abs(v), DEADZONE, 512, 1, JOY_MAX_STEP);
      moveTo(i, target[i] + (v > 0 ? step : -step) * dir[i]);
      moved = true;
    }
  }
  return moved;
}

void printAngles() {
  if (millis() - lastPrint < 300) return;
  lastPrint = millis();
  Serial.print(F("Goc: "));
  for (byte i = 0; i < 4; i++) {
    Serial.print(angle[i]);
    Serial.print(i < 3 ? ", " : "\n");
  }
}

// ---------- Cảm biến siêu âm (đo bằng mm) ----------
int readMm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long d = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (d == 0) return 9999;                             // không có vật
  return (int)((d * 10UL) / 58UL);                     // 58 us = 1 cm
}

int zoneOf(int mm) {
  for (byte z = 0; z < NUM_ZONES; z++) {
    if (mm >= ZONE_LO[z] && mm < ZONE_HI[z]) return z;
  }
  return -1;   // ngoài các ô
}

bool senseDue() {
  if (millis() - lastSense < SENSE_INTERVAL) return false;
  lastSense = millis();
  return true;
}

// Trung vị của NUM_SAMPLES lần đo: sắp xếp rồi lấy giá trị ở giữa
int medianOf(const int *v) {
  int t[NUM_SAMPLES];
  for (byte i = 0; i < NUM_SAMPLES; i++) t[i] = v[i];
  for (byte i = 1; i < NUM_SAMPLES; i++) {
    int k = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[NUM_SAMPLES / 2];
}

// ---------- Máy trạng thái tự động ----------
void runAuto() {
  switch (state) {

    case S_IDLE:
      if (millis() - lastManual < MANUAL_LOCK_MS) break;

      if (needStart) {                             // vừa chỉnh tay xong: về vị trí mở đầu trước
        needStart = false;
        buildStartScript();
        state = S_RUN;
        Serial.println(F("Ve vi tri mo dau"));
        break;
      }

      if (!senseDue()) break;
      samples[sampleCount++] = readMm();
      if (sampleCount >= NUM_SAMPLES) {            // đủ 5 lần đo: chọn kết quả
        sampleCount = 0;
        int mm = medianOf(samples);                // giá trị ở giữa
        int z = zoneOf(mm);
        byte agree = 0;                            // bao nhiêu lần đo cùng ô với kết quả
        for (byte i = 0; i < NUM_SAMPLES; i++) if (zoneOf(samples[i]) == z) agree++;
        if (DEBUG) {
          Serial.print(F("Do: "));
          for (byte i = 0; i < NUM_SAMPLES; i++) { Serial.print(samples[i]); Serial.print(' '); }
          Serial.print(F("-> giua="));  Serial.print(mm);
          Serial.print(F(" mm, o="));    Serial.print(z + 1);
          Serial.print(F(" ("));         Serial.print(agree);
          Serial.print('/');             Serial.print(NUM_SAMPLES);
          Serial.println(F(" lan dong y)"));
        }
        if (z >= 0 && agree >= MIN_AGREE) {
          Serial.print(F("Phat hien vat o o "));
          Serial.println(z + 1);
          buildPickScript(z);
          state = S_RUN;
        }
      }
      break;

    case S_RUN:
      if (runScript()) {
        if (pickRun) {
          clearCount = 0;
          state = S_WAIT_CLEAR;
        } else {
          sampleCount = 0;
          state = S_IDLE;
        }
      }
      break;

    case S_WAIT_CLEAR:
      if (!senseDue()) break;
      if (zoneOf(readMm()) < 0) clearCount++;
      else clearCount = 0;
      if (clearCount >= CLEAR_N) {
        sampleCount = 0;
        state = S_IDLE;
      }
      break;
  }
}

// ---------- Chương trình chính ----------
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Đo tâm joystick: đừng chạm vào cần khi vừa bật nguồn
  for (byte i = 0; i < 4; i++) {
    long sum = 0;
    for (byte n = 0; n < 20; n++) { sum += analogRead(joyPins[i]); delay(5); }
    center[i] = sum / 20;
  }

  // Bật nguồn: đưa servo thẳng về tư thế mở đầu
  for (byte i = 0; i < 4; i++) {
    angle[i]  = constrain(START_POSE[i], minA[i], maxA[i]);
    target[i] = angle[i];
    servos[i].attach(servoPins[i]);
    servos[i].write(angle[i]);
  }
  Serial.println(F("San sang - dang o vi tri mo dau"));
}

void loop() {
  unsigned long now = millis();

  // Nhịp cập nhật servo và joystick cố định, không phụ thuộc thời gian đo cảm biến
  if (now - lastServo >= SERVO_INTERVAL) {
    lastServo = now;
    if (handleJoystick()) {        // gạt joystick: hủy kịch bản, tạm dừng tự động
      lastManual = now;
      state = S_IDLE;
      scriptClear();
      needStart = true;            // hết thời gian khóa sẽ tự về mở đầu
      sampleCount = 0;
      printAngles();
    }
    updateServos();
  }

  runAuto();
}