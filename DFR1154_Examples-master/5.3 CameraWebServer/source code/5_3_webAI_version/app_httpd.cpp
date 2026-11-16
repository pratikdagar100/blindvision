// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// Enable LED FLASH setting
#define CONFIG_LED_ILLUMINATOR_ENABLED 1

// LED FLASH setup
#if CONFIG_LED_ILLUMINATOR_ENABLED

#define LED_LEDC_GPIO            47  //configure LED pin
#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;

#endif

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en) {  // Turn LED On or Off
  int duty = en ? led_duty : 0;
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_LEDC_GPIO, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  log_i("Set LED intensity to %d", duty);
}
#endif

static esp_err_t bmp_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();
#endif
  fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/x-windows-bmp");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);
  esp_camera_fb_return(fb);
  if (!converted) {
    log_e("BMP Conversion failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  res = httpd_resp_send(req, (const char *)buf, buf_len);
  free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();
#endif
  log_i("BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
  return res;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
  enable_led(true);
  vTaskDelay(150 / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
  fb = esp_camera_fb_get();              // or it won't be visible in the frame. A better way to do this is needed.
  enable_led(false);
#else
  fb = esp_camera_fb_get();
#endif

  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  size_t fb_len = 0;
#endif
  if (fb->format == PIXFORMAT_JPEG) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = fb->len;
#endif
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = jchunk.len;
#endif
  }
  esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
#endif
  log_i("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = true;
  enable_led(true);
#endif

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }

    // Cap at ~20 fps: stops frames piling up in WiFi buffers (that is what causes lag)
    {
      int64_t spent_ms = (esp_timer_get_time() - last_frame) / 1000;
      if (spent_ms < 50) {
        vTaskDelay((50 - spent_ms) / portTICK_PERIOD_MS);
      }
    }

    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;

    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time,
      1000.0 / avg_frame_time
    );
  }

#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = false;
  enable_led(false);
#endif

  return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      *obuf = buf;
      return ESP_OK;
    }
    free(buf);
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  char variable[32];
  char value[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK || httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);
  log_i("%s = %d", variable, val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (!strcmp(variable, "quality")) {
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "contrast")) {
    res = s->set_contrast(s, val);
  } else if (!strcmp(variable, "brightness")) {
    res = s->set_brightness(s, val);
  } else if (!strcmp(variable, "saturation")) {
    res = s->set_saturation(s, val);
  } else if (!strcmp(variable, "gainceiling")) {
    res = s->set_gainceiling(s, (gainceiling_t)val);
  } else if (!strcmp(variable, "colorbar")) {
    res = s->set_colorbar(s, val);
  } else if (!strcmp(variable, "awb")) {
    res = s->set_whitebal(s, val);
  } else if (!strcmp(variable, "agc")) {
    res = s->set_gain_ctrl(s, val);
  } else if (!strcmp(variable, "aec")) {
    res = s->set_exposure_ctrl(s, val);
  } else if (!strcmp(variable, "hmirror")) {
    res = s->set_hmirror(s, val);
  } else if (!strcmp(variable, "vflip")) {
    res = s->set_vflip(s, val);
  } else if (!strcmp(variable, "awb_gain")) {
    res = s->set_awb_gain(s, val);
  } else if (!strcmp(variable, "agc_gain")) {
    res = s->set_agc_gain(s, val);
  } else if (!strcmp(variable, "aec_value")) {
    res = s->set_aec_value(s, val);
  } else if (!strcmp(variable, "aec2")) {
    res = s->set_aec2(s, val);
  } else if (!strcmp(variable, "dcw")) {
    res = s->set_dcw(s, val);
  } else if (!strcmp(variable, "bpc")) {
    res = s->set_bpc(s, val);
  } else if (!strcmp(variable, "wpc")) {
    res = s->set_wpc(s, val);
  } else if (!strcmp(variable, "raw_gma")) {
    res = s->set_raw_gma(s, val);
  } else if (!strcmp(variable, "lenc")) {
    res = s->set_lenc(s, val);
  } else if (!strcmp(variable, "special_effect")) {
    res = s->set_special_effect(s, val);
  } else if (!strcmp(variable, "wb_mode")) {
    res = s->set_wb_mode(s, val);
  } else if (!strcmp(variable, "ae_level")) {
    res = s->set_ae_level(s, val);
  }
#if CONFIG_LED_ILLUMINATOR_ENABLED
  else if (!strcmp(variable, "led_intensity")) {
    led_duty = val;
    if (isStreaming) {
      enable_led(true);
    }
  }
#endif
  else {
    log_i("Unknown command: %s", variable);
    res = -1;
  }

  if (res < 0) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask) {
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg = 0x3400; reg < 0x3406; reg += 2) {
      p += print_reg(p, s, reg, 0xFFF);  //12 bit
    }
    p += print_reg(p, s, 0x3406, 0xFF);

    p += print_reg(p, s, 0x3500, 0xFFFF0);  //16 bit
    p += print_reg(p, s, 0x3503, 0xFF);
    p += print_reg(p, s, 0x350a, 0x3FF);   //10 bit
    p += print_reg(p, s, 0x350c, 0xFFFF);  //16 bit

    for (int reg = 0x5480; reg <= 0x5490; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5380; reg <= 0x538b; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5580; reg < 0x558a; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }
    p += print_reg(p, s, 0x558a, 0x1FF);  //9 bit
  } else if (s->id.PID == OV2640_PID) {
    p += print_reg(p, s, 0xd3, 0xFF);
    p += print_reg(p, s, 0x111, 0xFF);
    p += print_reg(p, s, 0x132, 0xFF);
  }

  p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
  p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#if CONFIG_LED_ILLUMINATOR_ENABLED
  p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
  p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _xclk[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int xclk = atoi(_xclk);
  log_i("Set XCLK: %d MHz", xclk);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];
  char _val[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK
      || httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  int val = atoi(_val);
  log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_reg(s, reg, mask, val);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->get_reg(s, reg, mask);
  if (res < 0) {
    return httpd_resp_send_500(req);
  }
  log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

  char buffer[20];
  const char *val = itoa(res, buffer, 10);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char *key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK) {
    return def;
  }
  return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int bypass = parse_get_var(buf, "bypass", 0);
  int mul = parse_get_var(buf, "mul", 0);
  int sys = parse_get_var(buf, "sys", 0);
  int root = parse_get_var(buf, "root", 0);
  int pre = parse_get_var(buf, "pre", 0);
  int seld5 = parse_get_var(buf, "seld5", 0);
  int pclken = parse_get_var(buf, "pclken", 0);
  int pclk = parse_get_var(buf, "pclk", 0);
  free(buf);

  log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int startX = parse_get_var(buf, "sx", 0);
  int startY = parse_get_var(buf, "sy", 0);
  int endX = parse_get_var(buf, "ex", 0);
  int endY = parse_get_var(buf, "ey", 0);
  int offsetX = parse_get_var(buf, "offx", 0);
  int offsetY = parse_get_var(buf, "offy", 0);
  int totalX = parse_get_var(buf, "tx", 0);
  int totalY = parse_get_var(buf, "ty", 0);  // codespell:ignore totaly
  int outputX = parse_get_var(buf, "ox", 0);
  int outputY = parse_get_var(buf, "oy", 0);
  bool scale = parse_get_var(buf, "scale", 0) == 1;
  bool binning = parse_get_var(buf, "binning", 0) == 1;
  free(buf);

  log_i(
    "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY,
    totalX, totalY, outputX, outputY, scale, binning  // codespell:ignore totaly
  );
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);  // codespell:ignore totaly
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// BlindVision sensor values are defined in the main .ino file.
extern bool tofReady;
extern bool tofValid;
extern uint16_t distanceMM;
extern unsigned long lastDistanceUpdate;

static esp_err_t blindvision_distance_handler(httpd_req_t *req) {
  char json[220];
  bool valid = tofReady && tofValid && (millis() - lastDistanceUpdate < 1500);
  unsigned long age = valid ? millis() - lastDistanceUpdate : 0;
  snprintf(json, sizeof(json),
           "{\"ready\":%s,\"valid\":%s,\"distance_mm\":%u,\"distance_cm\":%.1f,\"age_ms\":%lu}",
           tofReady ? "true" : "false", valid ? "true" : "false",
           valid ? distanceMM : 0, valid ? distanceMM / 10.0 : 0.0, age);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// ---------------- OFFLINE AI FILES (stored in ESP flash) ----------------
// TensorFlow.js + COCO-SSD model + sam-js are served from the ESP itself,
// so detection works with NO internet. Browser caches them after first load.
#include "src/ai_assets.h"

static esp_err_t ai_asset_handler(httpd_req_t *req) {
  const char *uri = req->uri;
  size_t ulen = strcspn(uri, "?");

  for (size_t i = 0; i < ai_assets_count; i++) {
    const ai_asset_t *a = &ai_assets[i];
    if (strlen(a->uri) == ulen && strncmp(a->uri, uri, ulen) == 0) {
      httpd_resp_set_type(req, a->type);
      if (a->gzip) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
      }
      httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000, immutable");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      return httpd_resp_send(req, (const char *)a->data, a->len);
    }
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

// ---------------- AI SNAPSHOT ----------------
// Single JPEG for the browser AI. Copied out of the camera buffer at once so the
// camera keeps running smoothly while the picture is sent over WiFi.
static esp_err_t frame_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  size_t len = fb->len;
  uint8_t *copy = (uint8_t *)ps_malloc(len);
  if (!copy) {
    esp_camera_fb_return(fb);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  memcpy(copy, fb->buf, len);
  esp_camera_fb_return(fb);

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)copy, len);
  free(copy);
  return res;
}

// ---------------- ON-BOARD SPEAKER (speaker.cpp) ----------------
bool speakerIsBusy();
bool speakerPlayU8(uint8_t *buf, size_t len, int volume);

#define SPEAK_MAX_BYTES (22050 * 8)  // max 8 s of 8-bit audio

// POST /speak?vol=0..100  body = raw 8-bit unsigned mono PCM @ 22050 Hz
static esp_err_t speak_handler(httpd_req_t *req) {
  size_t len = req->content_len;

  if (len == 0 || len > SPEAK_MAX_BYTES) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad audio size");
    return ESP_FAIL;
  }

  int volume = 70;
  char query[32];
  char val[8];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK && httpd_query_key_value(query, "vol", val, sizeof(val)) == ESP_OK) {
    volume = atoi(val);
  }

  uint8_t *buf = (uint8_t *)ps_malloc(len);
  if (!buf) buf = (uint8_t *)malloc(len);
  if (!buf) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  size_t got = 0;
  while (got < len) {
    int r = httpd_req_recv(req, (char *)buf + got, len - got);
    if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (r <= 0) {
      free(buf);
      return ESP_FAIL;
    }
    got += r;
  }

  bool ok = speakerPlayU8(buf, len, volume);  // takes ownership of buf
  Serial.printf("[speak] %u bytes, vol %d -> %s\n", (unsigned)len, volume, ok ? "playing" : "busy/not ready");

  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (!ok) {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_send(req, "busy", HTTPD_RESP_USE_STRLEN);
  }
  return httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t index_handler(httpd_req_t *req) {
  static const char html[] = R"rawliteral(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BlindVision</title><style>
*{box-sizing:border-box}body{margin:0;padding:18px;background:#101820;color:#f4f7fa;font-family:Arial,sans-serif;text-align:center}
h1{letter-spacing:3px;margin:8px 0}.wrap{max-width:900px;margin:auto}.panel{background:#1d2b36;border-radius:16px;padding:16px;margin:14px auto}
.view{position:relative;width:100%;max-width:850px;margin:auto}
img{width:100%;border-radius:12px;background:#000;display:block}
#ov{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
.distance{font-size:clamp(58px,14vw,100px);font-weight:700;color:#00e5a0;line-height:1.1}
.small{color:#aebbc5}#state{color:#00e5a0;font-weight:600}
#det{font-size:22px;font-weight:700;color:#ffd84d;min-height:28px;margin:12px 0}
.ctl{display:flex;flex-wrap:wrap;gap:12px;justify-content:center;align-items:center;font-size:15px}
button{font-size:20px;padding:14px 26px;border:0;border-radius:12px;background:#00e5a0;color:#06251a;font-weight:700;cursor:pointer}
button.on{background:#ff5d5d;color:#fff}
</style></head><body><div class="wrap"><h1>BLINDVISION</h1><p class="small">AI ASSISTIVE VISION SYSTEM</p>
<div class="panel"><h3>LIVE CAMERA + AI DETECTION</h3>
<div class="view"><img id="cam" alt="Camera stream"><canvas id="ov"></canvas></div>
<p id="det">Loading AI model...</p>
<p id="dbg" class="small"></p>
<div class="ctl"><button id="go">&#9654; START VOICE</button><button id="test" style="background:#4dc3ff">&#128266; TEST VOICE</button></div>
<div class="ctl" style="margin-top:12px">
<label><input type="checkbox" id="vPhone" checked> Phone voice</label>
<label><input type="checkbox" id="vEsp" checked> ESP speaker</label>
<label>ESP volume <input type="range" id="vol" min="0" max="100" value="80"></label>
<label>Confidence <input type="range" id="thr" min="20" max="90" value="40"></label>
</div></div>
<div class="panel"><h3>FORWARD DISTANCE</h3><div class="distance" id="cm">--</div><div class="small">CENTIMETRES</div><p id="mm" class="small">-- mm</p><p id="state">Connecting to sensor...</p></div>
</div>
<script>
const $=id=>document.getElementById(id);
const cam=$('cam'),ov=$('ov'),ctx=ov.getContext('2d');
const DW=640;                                  // AI input width (model works at 300x300 internally)
const dc=document.createElement('canvas'),dctx=dc.getContext('2d');
// Smooth HD video for display only - starts immediately, reconnects by itself
const STREAM='http://'+location.hostname+':81/stream';
cam.onerror=()=>setTimeout(()=>{cam.src=STREAM+'?r='+Date.now();},1500);
cam.src=STREAM;

let tofOk=false,model=null,last=[],lastAt=0,dist=null,voiceOn=false,sam=null,backend='',fps=0,emptySince=Date.now(),lastCheck=0;

/* ---------- distance sensor ---------- */
async function updateDistance(){try{const r=await fetch('/distance?t='+Date.now(),{cache:'no-store'});const d=await r.json();tofOk=!!d.ready;
if(d.ready&&d.valid){dist=Math.round(d.distance_cm);$('cm').textContent=Number(d.distance_cm).toFixed(1);$('mm').textContent=d.distance_mm+' mm';$('state').textContent='LIVE • '+d.age_ms+' ms ago';}
else{dist=null;$('cm').textContent='--';$('mm').textContent='-- mm';$('state').textContent=d.ready?'Waiting for valid measurement':'VL53L4CD not detected';}}
catch(e){dist=null;$('state').textContent='Sensor connection lost';}}
updateDistance();setInterval(updateDistance,300);

/* ---------- object detection (COCO-SSD, runs in this browser, model served by the ESP) ---------- */
const VEH=['bicycle','car','motorcycle','airplane','bus','train','truck','boat'];
const ANI=['bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe'];
const NICE={'tv':'TV','cell phone':'phone','dining table':'table','potted plant':'plant','motorcycle':'motorbike'};
const name=c=>NICE[c]||c;
const color=c=>c==='person'?'#00e5a0':VEH.includes(c)?'#ff9f1c':ANI.includes(c)?'#ff4fd8':'#4dc3ff';
function plural(w){return w==='person'?'people':/(s|sh|ch|x)$/.test(w)?w+'es':w+'s';}
function summary(p){
  const g={};p.forEach(o=>{const a=o.bbox[2]*o.bbox[3];g[o.class]=g[o.class]||{n:0,a:0};g[o.class].n++;g[o.class].a+=a;});
  return Object.keys(g).sort((x,y)=>g[y].a-g[x].a)   // biggest / closest first
    .map(c=>g[c].n>1?g[c].n+' '+plural(name(c)):name(c)).join(', ');
}

// Fetch a real JPEG snapshot from the ESP (reliable on every browser), shrink it for the AI
async function grab(){
  const r=await fetch('/frame?t='+Date.now(),{cache:'no-store'});
  if(!r.ok)throw new Error('frame '+r.status);
  const bmp=await createImageBitmap(await r.blob());
  const h=Math.round(DW*bmp.height/bmp.width);
  if(dc.width!==DW||dc.height!==h){dc.width=DW;dc.height=h;}
  dctx.drawImage(bmp,0,0,DW,h);bmp.close();
  return dc;
}

function draw(p,w,h){
  if(ov.width!==w||ov.height!==h){ov.width=w;ov.height=h;}
  ctx.clearRect(0,0,w,h);
  const fs=Math.max(12,Math.round(w/30));
  ctx.lineWidth=3;ctx.font='bold '+fs+'px Arial';
  p.forEach(o=>{const[x,y,bw,bh]=o.bbox,c=color(o.class);
    ctx.strokeStyle=c;ctx.strokeRect(x,y,bw,bh);
    const t=name(o.class)+' '+Math.round(o.score*100)+'%',tw=ctx.measureText(t).width+8,th=fs+6,ty=Math.max(0,y-th);
    ctx.fillStyle=c;ctx.fillRect(x,ty,tw,th);ctx.fillStyle='#000';ctx.fillText(t,x+4,ty+fs);});
  $('det').textContent=p.length?summary(p):'Nothing detected';
}

const topScore=a=>a.length?Math.max(...a.map(o=>o.score)):0;

// Some phone GPUs give wrong AI results in WebGL. Compare once with the exact CPU result.
async function verifyBackend(img,gpuRes){
  if(tf.getBackend()==='cpu')return;
  lastCheck=Date.now();
  await tf.setBackend('cpu');
  const cpuRes=await model.detect(img,20,0.2);
  if(Math.abs(topScore(cpuRes)-topScore(gpuRes))>0.15||(cpuRes.length&&!gpuRes.length)){
    backend='cpu (GPU gave wrong results)';console.warn('WebGL mismatch',gpuRes,cpuRes);
  }else{await tf.setBackend('webgl');backend='webgl';}
}

async function loop(){
  const t0=performance.now();
  try{
    const img=await grab();
    const all=await model.detect(img,20,0.2);
    if(!lastCheck)await verifyBackend(img,all);
    const thr=$('thr').value/100,p=all.filter(o=>o.score>=thr);
    draw(p,img.width,img.height);last=p;lastAt=Date.now();
    if(p.length)emptySince=Date.now();
    else if(Date.now()-emptySince>15000&&Date.now()-lastCheck>30000&&tf.getBackend()!=='cpu')await verifyBackend(img,all);
    const best=all.length?all.reduce((a,b)=>a.score>b.score?a:b):null;
    fps=fps*0.8+200/(performance.now()-t0);
    $('dbg').textContent='AI: '+backend+' • '+fps.toFixed(1)+' fps'+(best?' • best guess: '+name(best.class)+' '+Math.round(best.score*100)+'%':'');
  }catch(e){console.warn(e);$('dbg').textContent='AI error: '+e.message;await new Promise(r=>setTimeout(r,500));}
  setTimeout(loop,60);
}

// Load AI libraries in the background (camera + distance never wait for them)
function loadScript(src,tries=3){
  return new Promise((ok,fail)=>{
    const s=document.createElement('script');s.src=src;s.async=false;
    s.onload=ok;
    s.onerror=()=>{s.remove();tries>1?setTimeout(()=>loadScript(src,tries-1).then(ok,fail),1500):fail(new Error('could not load '+src));};
    document.body.appendChild(s);
  });
}
// Download the 9 MB model once with a progress display; the browser keeps it in its cache
async function prefetchModel(){
  try{
    const r=await fetch('/ai/w.bin');const total=+r.headers.get('Content-Length')||9017328;
    const rd=r.body.getReader();let got=0;
    for(;;){const{done,value}=await rd.read();if(done)break;got+=value.length;
      $('det').textContent='Downloading AI model from ESP: '+Math.round(100*got/total)+'%';}
  }catch(e){console.warn('prefetch',e);}
}
async function startAI(){
  try{
    $('det').textContent='Loading AI libraries...';
    await loadScript('/ai/tf.min.js');
    await loadScript('/ai/coco-ssd.min.js');
    await loadScript('/ai/samjs.min.js');
  }catch(e){$('det').textContent='AI library failed to load from the ESP - reload the page.';return;}
  await prefetchModel();
  loadModel();
}

async function loadModel(){
  if(typeof cocoSsd==='undefined'){$('det').textContent='AI library failed to load from the ESP - reload the page.';return;}
  $('det').textContent='Starting AI...';
  try{
    try{await tf.setBackend('webgl');}catch(e){await tf.setBackend('cpu');}
    await tf.ready();backend=tf.getBackend();
    try{model=await cocoSsd.load({base:'lite_mobilenet_v2',modelUrl:'/ai/model.json'});}   // offline, from ESP flash
    catch(e1){console.warn('Local model failed, trying online',e1);model=await cocoSsd.load({base:'lite_mobilenet_v2'});}
    $('det').textContent='AI ready';loop();
  }catch(e){$('det').textContent='AI model failed to load: '+e.message;}
}
startAI();

/* ---------- voice: phone + ESP speaker, every 3 seconds ---------- */
function numWords(n){
  const a=['zero','one','two','three','four','five','six','seven','eight','nine','ten','eleven','twelve','thirteen','fourteen','fifteen','sixteen','seventeen','eighteen','nineteen'];
  const t=['','','twenty','thirty','forty','fifty','sixty','seventy','eighty','ninety'];
  if(n<20)return a[n];if(n<100)return t[Math.floor(n/10)]+(n%10?' '+a[n%10]:'');
  return a[Math.floor(n/100)]+' hundred'+(n%100?' '+numWords(n%100):'');
}
let enVoice=null;
function pickVoice(){const v=speechSynthesis.getVoices();enVoice=v.find(x=>/^en[-_]/i.test(x.lang)&&x.localService)||v.find(x=>/^en/i.test(x.lang))||null;}
if('speechSynthesis' in window){pickVoice();speechSynthesis.onvoiceschanged=pickVoice;setInterval(()=>{if(speechSynthesis.paused)speechSynthesis.resume();},5000);}
function phoneSay(s){
  if(!('speechSynthesis' in window)){$('dbg').textContent='This browser has no speech support';return;}
  const ss=speechSynthesis;
  if(ss.speaking||ss.pending)ss.cancel();
  setTimeout(()=>{const u=new SpeechSynthesisUtterance(s);u.lang='en-US';u.rate=1.05;if(enVoice)u.voice=enVoice;ss.speak(u);},80);
}
function espSay(s){
  if(typeof SamJs==='undefined')return;
  sam=sam||new SamJs({speed:72,pitch:64,throat:128,mouth:128});
  const txt=s.replace(/\bTV\b/g,'tee vee').replace(/\d+/g,m=>numWords(+m)).replace(/[^a-zA-Z ,.]/g,' ').slice(0,200);
  const b=sam.buf8(txt);if(!b||!b.length)return;
  fetch('/speak?vol='+$('vol').value,{method:'POST',body:b,headers:{'Content-Type':'application/octet-stream'}}).catch(()=>{});
}
function say(s){if($('vPhone').checked)phoneSay(s);if($('vEsp').checked)espSay(s);}

setInterval(()=>{
  if(!voiceOn||!model||Date.now()-lastAt>3000)return;
  // distance is read at the moment of speaking (sensor is polled every 300 ms)
  const dtxt=dist!==null?'distance '+dist+' centimetres':(tofOk?'distance more than one metre':'');
  let s='';
  if(last.length){s=summary(last)+(dtxt?'. '+dtxt:'');}
  else if(dist!==null){s='Obstacle, '+dtxt;}
  if(s)say(s);
},3000);

$('go').onclick=()=>{
  voiceOn=!voiceOn;
  $('go').classList.toggle('on',voiceOn);
  $('go').innerHTML=voiceOn?'&#9632; STOP VOICE':'&#9654; START VOICE';
  if(voiceOn)say('Voice on');else if('speechSynthesis' in window)speechSynthesis.cancel();
};
$('test').onclick=()=>say('Blind vision ready');
</script></body></html>)rawliteral";
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;
  config.uri_match_fn = httpd_uri_match_wildcard;  // for /ai/*
  config.send_wait_timeout = 15;                   // big AI files on slow phones
  config.lru_purge_enable = true;                  // drop stale idle connections instead of hanging

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t bmp_uri = {
    .uri = "/bmp",
    .method = HTTP_GET,
    .handler = bmp_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t xclk_uri = {
    .uri = "/xclk",
    .method = HTTP_GET,
    .handler = xclk_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t reg_uri = {
    .uri = "/reg",
    .method = HTTP_GET,
    .handler = reg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t greg_uri = {
    .uri = "/greg",
    .method = HTTP_GET,
    .handler = greg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t pll_uri = {
    .uri = "/pll",
    .method = HTTP_GET,
    .handler = pll_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t win_uri = {
    .uri = "/resolution",
    .method = HTTP_GET,
    .handler = win_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  ra_filter_init(&ra_filter, 20);

  log_i("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_uri_t distance_uri = { .uri = "/distance", .method = HTTP_GET, .handler = blindvision_distance_handler, .user_ctx = NULL };
    httpd_register_uri_handler(camera_httpd, &distance_uri);
    httpd_uri_t speak_uri = { .uri = "/speak", .method = HTTP_POST, .handler = speak_handler, .user_ctx = NULL };
    httpd_register_uri_handler(camera_httpd, &speak_uri);
    httpd_uri_t ai_uri = { .uri = "/ai/*", .method = HTTP_GET, .handler = ai_asset_handler, .user_ctx = NULL };
    httpd_register_uri_handler(camera_httpd, &ai_uri);
    httpd_uri_t frame_uri = { .uri = "/frame", .method = HTTP_GET, .handler = frame_handler, .user_ctx = NULL };
    httpd_register_uri_handler(camera_httpd, &frame_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);

    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash(int pin) {
#if CONFIG_LED_ILLUMINATOR_ENABLED
  ledcAttach(pin, 5000, 8);
#else
  log_i("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif
}
