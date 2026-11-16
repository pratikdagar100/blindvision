
#include "esp_camera.h"
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <VL53L4CD.h>

// =====================================================
// BLINDVISION MASTER FIRMWARE
// DFRobot ESP32-S3 AI Camera + VL53L4CD
// Combined camera stream + live distance webpage
// =====================================================


// ---------------- CAMERA PINS ----------------

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     5
#define Y9_GPIO_NUM       4
#define Y8_GPIO_NUM       6
#define Y7_GPIO_NUM       7
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       17
#define Y4_GPIO_NUM       21
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       16
#define VSYNC_GPIO_NUM    1
#define HREF_GPIO_NUM     2
#define PCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     8
#define SIOC_GPIO_NUM     9
#define LED_GPIO_NUM      47


// ---------------- TOF CONFIGURATION ----------------

#define TOF_SDA           43
#define TOF_SCL           44
#define TOF_ADDRESS       0x29

#define TOF_INTERVAL      10

VL53L4CD tof;

bool tofReady = false;
bool tofValid = false;

uint16_t distanceMM = 0;

unsigned long lastToFRead = 0;
unsigned long lastDistanceUpdate = 0;


// ---------------- SYSTEM VARIABLES ----------------

Preferences preferences;

bool wifiReady = false;
bool cameraReady = false;

uint8_t serialDistanceCount = 0;
bool serialStreamMessagePrinted = false;

// Original DFRobot camera functions
void startCameraServer();
void setupLedFlash(int pin);


// =====================================================
// WIFI
// =====================================================

bool connectToWiFi(const char* ssid, const char* password) {

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);

  Serial.printf("\nConnecting to WiFi: %s\n", ssid);

  int retries = 0;

  while (WiFi.status() != WL_CONNECTED && retries < 30) {

    delay(500);
    Serial.print(".");

    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("\nWiFi connected!");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    return true;
  }

  Serial.println("\nWiFi connection failed!");

  return false;
}


String readSerialInput(const char* message) {

  Serial.println(message);

  while (Serial.available()) {
    Serial.read();
  }

  while (Serial.available() == 0) {
    delay(10);
  }

  String value = Serial.readStringUntil('\n');
  value.trim();

  return value;
}


void initWiFi() {

  Serial.println("\n--- WIFI SETUP ---");

  preferences.begin("wifi", false);

  String savedSSID = preferences.getString("ssid", "");
  String savedPASS = preferences.getString("password", "");

  preferences.end();

  if (savedSSID.length() > 0) {

    Serial.println("Saved WiFi found.");

    wifiReady = connectToWiFi(
      savedSSID.c_str(),
      savedPASS.c_str()
    );
  }

  if (!wifiReady) {

    Serial.println("Enter WiFi credentials.");

    String ssid = readSerialInput("Enter SSID:");
    String password = readSerialInput("Enter Password:");

    if (ssid.length() == 0) {
      Serial.println("Empty SSID!");
      return;
    }

    wifiReady = connectToWiFi(
      ssid.c_str(),
      password.c_str()
    );

    if (wifiReady) {

      preferences.begin("wifi", false);

      preferences.putString("ssid", ssid);
      preferences.putString("password", password);

      preferences.end();

      Serial.println("Credentials saved.");
    }
  }
}


// =====================================================
// VL53L4CD
// =====================================================

void initToF() {

  Serial.println("\n--- VL53L4CD INITIALIZATION ---");

  Wire.begin(TOF_SDA, TOF_SCL, 100000);

  tof.setBus(&Wire);
  tof.setTimeout(500);

  if (!tof.init()) {

    tofReady = false;

    Serial.println("VL53L4CD NOT DETECTED!");

    return;
  }

  Serial.println("VL53L4CD detected!");

  if (!tof.setRangeTiming(50, 0)) {
    Serial.println("Timing configuration warning.");
  }

  tof.startContinuous();

  tofReady = true;

  Serial.println("Continuous ranging started.");
}



void readToF() {

  if (!tofReady) return;

  if (millis() - lastToFRead < TOF_INTERVAL) {
    return;
  }

  lastToFRead = millis();

  if (tof.dataReady()) {

    uint16_t reading = tof.read(false);

    if (tof.ranging_data.range_status == 0 &&
        reading > 0 &&
        reading <= 1300) {

      distanceMM = reading;
      tofValid = true;
      lastDistanceUpdate = millis();

      // Print only the first 3 valid readings
      if (serialDistanceCount < 3) {

        serialDistanceCount++;

        Serial.printf(
          "Distance %u: %u mm | %.1f cm\n",
          serialDistanceCount,
          distanceMM,
          distanceMM / 10.0
        );

        // After the third reading, print the stream message once
        if (serialDistanceCount == 3 &&
            !serialStreamMessagePrinted) {

          Serial.println();
          Serial.println("See next readings in live stream:");

          if (WiFi.status() == WL_CONNECTED) {
            Serial.print("http://");
            Serial.println(WiFi.localIP());
          }

          Serial.println();
          serialStreamMessagePrinted = true;
        }
      }

    } else {
      tofValid = false;
    }
  }
}



// =====================================================
// CAMERA INITIALIZATION
// =====================================================

bool initCamera() {

  Serial.println("\n--- CAMERA INITIALIZATION ---");

  camera_config_t config = {};

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

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_UXGA;

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  config.fb_location = CAMERA_FB_IN_PSRAM;

  config.jpeg_quality = 12;

  config.fb_count = 1;


  if (psramFound()) {

    config.jpeg_quality = 10;

    config.fb_count = 2;

    config.grab_mode = CAMERA_GRAB_LATEST;

  } else {

    config.frame_size = FRAMESIZE_SVGA;

    config.fb_location = CAMERA_FB_IN_DRAM;
  }


  esp_err_t error = esp_camera_init(&config);

  if (error != ESP_OK) {

    Serial.printf(
      "Camera initialization failed: 0x%x\n",
      error
    );

    return false;
  }


  sensor_t *sensor = esp_camera_sensor_get();

  if (sensor != NULL) {

    if (sensor->id.PID == OV3660_PID) {

      sensor->set_vflip(sensor, 1);
      sensor->set_brightness(sensor, 1);
      sensor->set_saturation(sensor, -2);
    }

    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  }

  Serial.println("Camera initialized successfully!");

  return true;
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  Serial.setDebugOutput(true);

  delay(1500);

  Serial.println();

  Serial.println("====================================");

  Serial.println("       BLINDVISION MASTER");

  Serial.println("====================================");


  // 1. Distance sensor
  initToF();


  // 2. Camera
  cameraReady = initCamera();

  if (cameraReady) {

    setupLedFlash(LED_GPIO_NUM);
  }


  // 3. WiFi
  initWiFi();


  // 4. Camera server
  if (cameraReady && wifiReady) {

    startCameraServer();

    Serial.println("Camera server started!");
  }


  // 6. Print addresses
  Serial.println();

  Serial.println("====================================");

  Serial.println("       BLINDVISION READY");

  Serial.println("====================================");


  if (wifiReady) {

    String ip = WiFi.localIP().toString();

    Serial.print("CAMERA: http://");
    Serial.println(ip);

    Serial.print("BLINDVISION LIVE PAGE: http://");
    Serial.println(ip);
    Serial.print("CAMERA STREAM: http://");
    Serial.print(ip);
    Serial.println(":81/stream");
    Serial.print("DISTANCE DATA: http://");
    Serial.print(ip);
    Serial.println("/distance");
  }


  Serial.printf(
    "Camera: %s\n",
    cameraReady ? "READY" : "FAILED"
  );

  Serial.printf(
    "VL53L4CD: %s\n",
    tofReady ? "READY" : "FAILED"
  );

  Serial.printf(
    "WiFi: %s\n",
    wifiReady ? "CONNECTED" : "DISCONNECTED"
  );

  Serial.println("====================================");

}


// =====================================================
// LOOP
// =====================================================

void loop() {

  readToF();

  delay(5);
}
