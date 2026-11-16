BLINDVISION - FINAL INTEGRATED FIRMWARE

FILES
1. BlindVision.ino       Main sketch (camera + Wi-Fi + VL53L4CD).
2. app_httpd.cpp         Modified DFRobot camera web server with combined page and /distance endpoint.

INSTALL
1. Make a copy of your working Arduino sketch folder first.
2. In the existing DFRobot camera sketch folder, replace the main .ino with BlindVision.ino (rename it to match the sketch folder, e.g. 5_3.ino).
3. Replace the existing app_httpd.cpp with this app_httpd.cpp.
4. KEEP the other original DFRobot files unchanged, especially camera_index.h and any other supplied camera files.
5. Install the same Pololu VL53L4CD library used by your currently working sensor sketch. Do not change the sensor library/API if your present version compiles and reads correctly.
6. Select ESP32S3 Dev Module; USB CDC On Boot Enabled; Flash Size 16MB; PSRAM OPI PSRAM; upload speed 115200.
7. Upload. Open Serial Monitor at 115200. If Wi-Fi credentials are not saved, enter SSID then password, each followed by Enter.
8. Connect your phone/PC to the same Wi-Fi network and open the IP printed as BLINDVISION LIVE PAGE.

EXPECTED
- Main page: http://BOARD-IP/
- MJPEG stream: http://BOARD-IP:81/stream
- Distance JSON: http://BOARD-IP/distance
The main page shows camera and distance together. No port 8082 server is used.

IMPORTANT
- This code retains your existing camera pin map and VL53L4CD read logic from the supplied working sketch.
- The camera stream is served on port 81 by the supplied DFRobot app_httpd.cpp.
- The distance endpoint returns stale readings as invalid after 1.5 seconds.
- The browser page is served on port 80.
- Do not overwrite camera_index.h or remove the original DFRobot support files.
- Test on a bench before mounting/wearing. A single ToF sensor measures only the direction it faces and is not a substitute for safe mobility aids.
