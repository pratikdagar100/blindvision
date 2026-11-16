
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

// On-board speaker (speaker.cpp)
bool initSpeaker();
bool speakerReady = false;


// =====================================================
// WIFI
// The ESP always creates its OWN hotspot with a fixed IP:
//     SSID: BlindVision   PASSWORD: blindvision123
//     Page: http://192.168.4.1   (bookmark this - it never changes)
// If home WiFi credentials were saved earlier it ALSO joins that
// network (optional, never blocks startup).
// =====================================================

#define AP_SSID     "BlindVision"
#define AP_PASSWORD "blindvision123"   // min 8 characters

IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);
IPAddress apLeaseStart(192, 168, 4, 2);
IPAddress apDNS(8, 8, 8, 8);           // DNS handed to phones (works through NAT)

bool apReady = false;
bool staReady = false;
bool naptReady = false;

bool startHotspot() {

  // DNS 8.8.8.8 is given to phones so websites resolve through the shared internet
  WiFi.softAPConfig(apIP, apGateway, apSubnet, apLeaseStart, apDNS);

  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Hotspot FAILED to start!");
    return false;
  }

  Serial.printf("Hotspot started: %s / %s\n", AP_SSID, AP_PASSWORD);
  Serial.print("Hotspot IP: ");
  Serial.println(WiFi.softAPIP());

  return true;
}

// Share the home/office WiFi internet (e.g. "ACIC desk") with hotspot clients
void enableInternetSharing() {

  if (!apReady || naptReady) return;

  naptReady = WiFi.AP.enableNAPT(true);

  Serial.printf("Internet sharing to hotspot: %s\n", naptReady ? "ON" : "FAILED");
}

bool connectToWiFi(const char* ssid, const char* password) {

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


void initWiFi() {

  Serial.println("\n--- WIFI SETUP ---");

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  // 1. Join home/office WiFi FIRST (gives internet; hotspot then uses same channel)
  preferences.begin("wifi", false);

  String savedSSID = preferences.getString("ssid", "");
  String savedPASS = preferences.getString("password", "");

  preferences.end();

  if (savedSSID.length() > 0) {

    Serial.println("Saved home WiFi found.");

    staReady = connectToWiFi(
      savedSSID.c_str(),
      savedPASS.c_str()
    );

  } else {

    Serial.println("No home WiFi saved - hotspot only.");
    Serial.println("To add one, type in Serial Monitor:  wifi YourSSID,YourPassword");
  }

  // 2. Own hotspot - always available, fixed IP 192.168.4.1
  apReady = startHotspot();

  // 3. Give hotspot clients internet through the home/office WiFi
  if (staReady) {
    enableInternetSharing();
  }

  wifiReady = apReady || staReady;
}


// Non-blocking: type  "wifi SSID,PASSWORD"  in Serial Monitor to save home WiFi
void checkSerialWiFiCommand() {

  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("wifi ")) return;

  line = line.substring(5);
  int comma = line.indexOf(',');

  if (comma <= 0) {
    Serial.println("Format: wifi SSID,PASSWORD");
    return;
  }

  String ssid = line.substring(0, comma);
  String password = line.substring(comma + 1);

  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();

  Serial.println("Home WiFi saved. Connecting...");

  staReady = connectToWiFi(ssid.c_str(), password.c_str());

  if (staReady) {
    enableInternetSharing();
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

          Serial.print("http://");
          Serial.println(WiFi.softAPIP());

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

    config.jpeg_quality = 12;   // HD with smaller files = less WiFi lag

    config.fb_count = 3;        // stream + AI snapshot + camera filling the next one

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
      sensor->set_hmirror(sensor, 1);   // picture was mirrored (text read backwards)
      sensor->set_brightness(sensor, 1);
      sensor->set_saturation(sensor, -2);
    }

    sensor->set_framesize(sensor, FRAMESIZE_HD);   // 1280 x 720
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


  // 3. Speaker (MAX98357)
  speakerReady = initSpeaker();


  // 4. WiFi (own hotspot + optional home WiFi)
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


  if (apReady) {

    Serial.printf("1) Connect phone to WiFi: %s  (password: %s)\n", AP_SSID, AP_PASSWORD);
    Serial.println("2) Open / bookmark:  http://192.168.4.1");
    Serial.println("   Stream: http://192.168.4.1:81/stream");
  }

  if (staReady) {

    Serial.print("Also on home WiFi: http://");
    Serial.println(WiFi.localIP());
  }


  Serial.printf(
    "Camera: %s\n",
    cameraReady ? "READY" : "FAILED"
  );

  Serial.printf(
    "Speaker: %s\n",
    speakerReady ? "READY" : "FAILED"
  );

  Serial.printf(
    "VL53L4CD: %s\n",
    tofReady ? "READY" : "FAILED"
  );

  Serial.printf(
    "Hotspot: %s | Home WiFi: %s | Internet on hotspot: %s\n",
    apReady ? "ON (192.168.4.1)" : "FAILED",
    staReady ? "CONNECTED" : "not connected",
    naptReady ? "YES" : "NO"
  );

  Serial.println("====================================");

}


// =====================================================
// LOOP
// =====================================================

void loop() {

  readToF();

  checkSerialWiFiCommand();

  // Home WiFi connected later (auto-reconnect)? Turn on internet sharing then.
  if (!naptReady && WiFi.status() == WL_CONNECTED) {
    staReady = true;
    enableInternetSharing();
  }

  delay(5);
}
