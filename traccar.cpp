#include "traccar.h"

/*
 * Snapshot copy of Traccar-related logic extracted from:
 * - telelogger.cpp (unique device ID + GNSS time/data formatting)
 * - teleclient.cpp (data upload to server)
 *
 * NOTE: Wrapped in #if 0 so this copy is not compiled or used.
 */

#if 0

void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif
  if (!gd || lastGPStime == gd->time) return false;
  if (gd->date) {
    // generate ISO time string
    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  }
  if (gd->lng == 0 && gd->lat == 0) {
    // coordinates not ready
    if (gd->date) {
      serial_log_printf(LOG_INFO, "[GNSS] %s", isoTime);
    }
    return false;
  }
  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {
    // invalid coordinates data
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float)); /* m */
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }

  serial_log_printf(LOG_INFO, "[GNSS] %.6f %.6f %dkm/h SATS:%u HDOP:%u Course:%u",
    gd->lat, gd->lng, (int)kph, gd->sat, gd->hdop, gd->heading);
  //serial_log_print(LOG_INFO, gd->errors);
  lastGPStime = gd->time;
  return true;
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
#if ENABLE_WIFI
  if ((wifi.connected() && wifi.state() != HTTP_CONNECTED) || cell.state() != HTTP_CONNECTED) {
#else
  if (cell.state() != HTTP_CONNECTED) {
#endif
    // reconnect if disconnected
    if (!connect(true)) {
      return false;
    }
  }

  char path[256];
  bool success = false;
  int len;
#if SERVER_PROTOCOL == PROTOCOL_HTTPS_GET
  if (gd && gd->ts) {
    len = snprintf(url, sizeof(url), "%s/push?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      SERVER_PATH, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    len = snprintf(url, sizeof(url), "%s/push?id=%s", SERVER_PATH, devid);
  }
  success = cell.send(METHOD_GET, SERVER_HOST, SERVER_PORT, url);
#else
  len = snprintf(path, sizeof(path), "%s/post/%s", SERVER_PATH, devid);
#if ENABLE_WIFI
  if (wifi.connected()) {
    serial_log_printf(LOG_INFO, "[WIFI] %s", path);
    success = wifi.send(METHOD_POST, path, packetBuffer, packetSize);
  }
  else
#endif
  {
    serial_log_printf(LOG_INFO, "[CELL] %s", path);
    success = cell.send(METHOD_POST, SERVER_HOST, SERVER_PORT, path, packetBuffer, packetSize);
  }
  len += packetSize;
#endif
  if (!success) {
    serial_log_print(LOG_INFO, "[HTTP] Connection closed");
    return false;
  } else {
    txBytes += len;
    txCount++;
  }

  // check response
  int recvBytes = 0;
  char* content = 0;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    content = wifi.receive(cell.getBuffer(), RECV_BUF_SIZE - 1, &recvBytes);
  }
  else
#endif
  {
    content = cell.receive(&recvBytes, HTTP_CONN_TIMEOUT);
  }
  if (!content) {
    // close connection on receiving timeout
    serial_log_print(LOG_INFO, "[HTTP] No response");
    return false;
  }
  serial_log_printf(LOG_INFO, "[HTTP] %s", content);
#if ENABLE_WIFI
  if ((wifi.connected() && wifi.code() == 200) || cell.code() == 200) {
#else
  if (cell.code() == 200) {
#endif
    // successful
    lastSyncTime = millis();
    rxBytes += recvBytes;
  }
  return true;
}

#endif
