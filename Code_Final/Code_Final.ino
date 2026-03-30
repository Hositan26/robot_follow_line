#include <WiFi.h>

// ====================================================================
// 1. CẤU HÌNH WIFI (Access Point cho robot)
// ====================================================================
const char* ssid = "Hositan";
const char* password = "12345679";
WiFiServer server(6868);
WiFiClient client;

// ====================================================================
// 2. CHÂN CẢM BIẾN (5 mắt TCRT5000)
// ====================================================================
#define S1 32
#define S2 33
#define S3 25
#define S4 26
#define S5 27
const int sensorPins[5] = { S1, S2, S3, S4, S5 };

// ====================================================================
// 3. CHÂN ĐỘNG CƠ VÀ PWM
// ====================================================================
#define IN1 19 // Bánh trái
#define IN2 21
#define IN3 17 // Bánh phải
#define IN4 18
#define PWM_FREQ 20000
#define PWM_RES 8

// ====================================================================
// 4. THAM SỐ BỘ ĐIỀU KHIỂN PD
// ====================================================================
float Kp = 33.0;      // Hệ số tỉ lệ
float Kd = 305.0;     // Hệ số vi phân
int baseSpeed = 70;   // Tốc độ cơ bản (0-255)

// ====================================================================
// 5. BIẾN TOÀN CỤC
// ====================================================================
float error = 0, lastError = 0;
int activeSensors = 0;     // Số cảm biến phát hiện vạch
int sensors[5];            // Giá trị từng cảm biến (0: thấy vạch, 1: không)
int turnState = 0;         // -1: rẽ trái, 0: thẳng, 1: rẽ phải
bool commandReceived = false;   // Cờ nhận lệnh từ PC

#define DEBUG 1            // Bật chế độ in debug (1) hoặc tắt (0)

// ====================================================================
// 6. CÁC HÀM ĐIỀU KHIỂN ĐỘNG CƠ (vi sai)
// ====================================================================
void moveMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Bánh trái (IN1, IN2)
  if (left >= 0) {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, left);
  } else {
    ledcWrite(IN1, -left);
    ledcWrite(IN2, 0);
  }

  // Bánh phải (IN3, IN4)
  if (right >= 0) {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, right);
  } else {
    ledcWrite(IN3, -right);
    ledcWrite(IN4, 0);
  }
}

// ====================================================================
// 7. ĐỌC CẢM BIẾN
// ====================================================================
void readSensors() {
  activeSensors = 0;
  for (int i = 0; i < 5; i++) {
    sensors[i] = digitalRead(sensorPins[i]);
    if (sensors[i] == 0) activeSensors++;   // 0 = phát hiện vạch đen
  }
}

// ====================================================================
// 8. TÍNH TOÁN SAI SỐ (error) DỰA TRÊN TRỌNG SỐ CẢM BIẾN
// ====================================================================
void calculateError() {
  float sum = 0;
  if (activeSensors > 0) {
    for (int i = 0; i < 5; i++) {
      if (sensors[i] == 0) sum += (i - 2);   // trọng số: -2, -1, 0, 1, 2
    }
    error = sum / activeSensors;
  } else {
    // Mất line: quay theo hướng lỗi cũ
    error = (lastError > 0) ? 3 : -3;
  }
}

// ====================================================================
// 9. XÁC ĐỊNH HƯỚNG RẼ TỪ SAI SỐ (dùng cho rẽ tự động)
// ====================================================================
void setTurnStateFromError() {
  if (error < 0) {
    turnState = -1;   // rẽ trái
  } else if (error > 0) {
    turnState = 1;    // rẽ phải
  } else {
    turnState = 0;    // đi thẳng
  }
}

// ====================================================================
// 10. KIỂM TRA GIAO LỘ
// ====================================================================
bool isIntersection() {
  return (activeSensors >= 3) && (sensors[0] == 0 || sensors[4] == 0);
}

// ====================================================================
// 11. RẼ TỰ ĐỘNG THEO PID (khi không có lệnh từ PC)
// ====================================================================
void performTurnPID() {
  // Bước 1: Tiến thẳng 1350ms
  moveMotors(baseSpeed, baseSpeed);
  unsigned long start = millis();
  while (millis() - start < 1350) {
    readSensors();
    if (sensors[2] == 0 && isIntersection() == false) {
      // Gặp line ngay khi tiến thẳng -> không cần quay
      turnState = 0;
      error = 0;
      lastError = 0;
      return;
    }
    delay(10);
  }

  // Bước 2: Quay theo hướng turnState, tối đa 2000ms
  int turnSpd = 105;
  moveMotors(turnState * turnSpd, -turnState * turnSpd - 5);
  start = millis();
  while (millis() - start < 2000) {
    readSensors();
    if (sensors[1] == 0 || sensors[2] == 0 || sensors[3] == 0) {
      // Đã gặp line -> kết thúc rẽ
      turnState = 0;
      error = 0;
      lastError = 0;
      return;
    }
    delay(10);
  }

  // Timeout: dừng xe
  moveMotors(0, 0);
  turnState = 0;
}

// ====================================================================
// 12. RẼ THEO LỆNH BIỂN BÁO (từ PC)
// ====================================================================
void performTurnSIGN() {
  // Bước 1: đi thẳng 1350ms
  unsigned long start = millis();
  while (millis() - start < 1350) {
    moveMotors(baseSpeed, baseSpeed);
    delay(10);
  }

  // Bước 2: quay 750ms theo hướng turnState
  unsigned long start1 = millis();
  while (millis() - start1 < 750) {
    int turnSpd = 105;
    moveMotors(turnState * turnSpd, -turnState * turnSpd - 5);
    delay(10);
  }

  // Bước 3: tiếp tục quay cho đến khi cảm biến S3 thấy line
  while (true) {
    readSensors();
    if (sensors[3] == 0) {
      turnState = 0;
      error = 0;
      lastError = 0;
      break;
    }
    int turnSpd = 105;
    moveMotors(turnState * turnSpd, -turnState * turnSpd - 5);
    delay(10);
  }

  // Dừng một chút
  moveMotors(0, 0);
  delay(50);
}

// ====================================================================
// 13. XỬ LÝ VẬT CẢN
// ====================================================================
void overcomeObstacle() {
  // Quay trái 400ms
  moveMotors(120, -120);
  delay(400);

  // Tiến thẳng cho đến khi cảm biến giữa S3 gặp line
  unsigned long start = millis();
  while (digitalRead(S3) == 1) {
    if (millis() - start > 2000) break;
    delay(10);
  }
  error = 0;
  lastError = 0;
}

// ====================================================================
// 14. XỬ LÝ BIỂN BÁO DỪNG (stop)
// ====================================================================
void handleStop() {
  // Căn chỉnh để đưa S3 vào line (tối đa 3 giây)
  unsigned long start = millis();
  const unsigned long timeout = 3000;

  while (millis() - start < timeout) {
    readSensors();
    if (sensors[2] == 0) break;   // đã căn chỉnh xong

    float alignError = 0;
    if (sensors[0] == 0 || sensors[1] == 0) {
      alignError = -1.5;   // lệch trái -> rẽ trái
    } else if (sensors[3] == 0 || sensors[4] == 0) {
      alignError = 1.5;    // lệch phải -> rẽ phải
    } else {
      alignError = 0;
    }

    int left = baseSpeed + (Kp * alignError);
    int right = baseSpeed - (Kp * alignError);
    moveMotors(left, right);
    delay(10);
  }

  // Dừng hoàn toàn
  moveMotors(0, 0);
  delay(10000);           // dừng 10 giây

  // Reset trạng thái
  error = 0;
  lastError = 0;
  turnState = 0;
}

// ====================================================================
// 15. XỬ LÝ LỆNH TCP TỪ PC
// ====================================================================
void processCommand(String cmd) {
  if (cmd == "leftSign") {
    turnState = -1;
  } else if (cmd == "rightSign") {
    turnState = 1;
  } else if (cmd == "obstacle") {
    overcomeObstacle();
    turnState = 0;
  } else if (cmd == "stopSign") {
    handleStop();
    turnState = 0;
  }
}

// ====================================================================
// 16. HÀM SETUP (khởi tạo)
// ====================================================================
void setup() {
  Serial.begin(115200);

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  server.begin();

  // Cấu hình chân cảm biến
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Cấu hình PWM cho các chân động cơ
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  ledcAttach(IN3, PWM_FREQ, PWM_RES);
  ledcAttach(IN4, PWM_FREQ, PWM_RES);

  Serial.println("Robot line follower ready (improved intersection detection).");
}

// ====================================================================
// 17. VÒNG LẶP CHÍNH
// ====================================================================
void loop() {
  // Đọc cảm biến và tính sai số
  readSensors();
  calculateError();

  // Quản lý kết nối TCP
  if (!client.connected()) {
    client = server.available();
    if (client) Serial.println("Client connected");
  }

  // Nhận lệnh từ PC
  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    if (cmd.length() > 0) {
      Serial.print("Command: ");
      Serial.println(cmd);
      commandReceived = true;
      processCommand(cmd);
    }
  }

  // Xử lý giao lộ: ưu tiên lệnh từ PC trước
  if (commandReceived && isIntersection()) {
    if (turnState != 0) {
      performTurnSIGN();       // rẽ theo biển báo
    }
    commandReceived = false;
  } else {
    // Không có lệnh hoặc chưa tới giao lộ -> rẽ tự động nếu gặp giao lộ
    if (isIntersection()) {
      setTurnStateFromError();
      if (turnState != 0) {
        performTurnPID();
      }
    }
  }

  // Điều khiển PD bám line (thực hiện liên tục)
  float P = error;
  float D = error - lastError;
  lastError = error;
  float motor = (Kp * P) + (Kd * D);
  moveMotors(baseSpeed + motor, baseSpeed - motor);

  // In thông tin debug nếu bật
  if (DEBUG) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("S:%d%d%d%d%d act:%d err:%.2f turn:%d\n",
                    sensors[0], sensors[1], sensors[2], sensors[3], sensors[4],
                    activeSensors, error, turnState);
    }
  }

  delay(10);
}