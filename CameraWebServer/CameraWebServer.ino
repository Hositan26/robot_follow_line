#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ====== Chọn model AI Thinker ======
#define CAMERA_MODEL_AI_THINKER

// ====== WiFi ======
const char* ssid = "Hositan";
const char* password = "12345679";

// ====== AI Thinker pinout ======
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

httpd_handle_t camera_httpd = NULL;

// ====== MJPEG Stream Handler với giới hạn FPS (tiết kiệm điện) ======
esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  char part_buf[64];
  httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  unsigned long last_frame_time = 0;
  // Tăng khoảng cách giữa các frame để giảm tải: 150ms = ~6.7 FPS
  const unsigned long frame_interval = 150000;  // micro giây

  while (true) {
    unsigned long now = micros();
    unsigned long wait = frame_interval - (now - last_frame_time);
    if (wait < frame_interval && wait > 0) {
      delayMicroseconds(wait);
    }
    last_frame_time = micros();

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf),
      "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      fb->len
    );

    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    esp_camera_fb_return(fb);
  }
  return ESP_OK;
}

// ====== Khởi tạo server ======
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_uri_t stream_uri = {
      .uri       = "/stream",
      .method    = HTTP_GET,
      .handler   = stream_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(camera_httpd, &stream_uri);
  }
}

// ====== Cấu hình camera (tiết kiệm năng lượng) ======
void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;   // 20MHz (có thể giảm xuống 10MHz nếu vẫn quá tải)
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    // Giảm độ phân giải và nén mạnh hơn để tiết kiệm băng thông & năng lượng
    config.frame_size = FRAMESIZE_QVGA;    // 320x240 (đủ để nhận biển báo)
    config.jpeg_quality = 40;              // 0-63, cao hơn = nén nhiều hơn (40 là cân bằng)
    config.fb_count = 1;                  // Chỉ 1 frame buffer để giảm tiêu thụ PSRAM
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 45;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);   // lật dọc (tuỳ chỉnh)
  s->set_hmirror(s, 0); // không soi gương
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  startCamera();

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("Open browser at: http://");
  Serial.println(WiFi.localIP());
  Serial.println("Stream URL: http://<IP>/stream");

  startCameraServer();
}

// ====== Loop ======
void loop() {
  delay(10000);  // không làm gì thêm
}