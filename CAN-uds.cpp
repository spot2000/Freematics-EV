/* CAN UDS (Unified Diagnostic Services) integration */
/* Below is functions to read and interpret UDS messages from CAN bus */
/* it uses functions from FreematicsOBD.cpp */
/* Functions to add: read UDS messages, interpret UDS responses */

#include "serial_logging.h"
#include <FreematicsPlus.h>
#include "CAN-uds.h"


// Add your UDS related functions here


extern COBD obd;  // or declare COBD obd globally

static bool hasAdapterErrorToken(const char* buf)
{
  if (!buf) return true;
  const char* tokens[] = {"NO DATA", "STOPPED", "ERROR", "UNABLE", "BUFFER FULL", "?"};
  for (size_t i = 0; i < sizeof(tokens) / sizeof(tokens[0]); i++) {
    if (strstr(buf, tokens[i])) return true;
  }
  return false;
}

static bool ensureAutoFlowControlEnabled()
{
  static uint32_t lastCheck = 0;
  uint32_t now = millis();
  if (now - lastCheck < 10000) {
    return true;
  }
  lastCheck = now;
  if (!obd.link) return false;
  char atbuf[64] = {0};
  int n = obd.link->sendCommand("ATCFC1\r", atbuf, sizeof(atbuf), 1000);
  if (!n || !strstr(atbuf, "OK")) {
    serial_log_print(LOG_INFO, "[UDS] ATCFC1 verify failed");
    serial_log_print(LOG_INFO, atbuf);
    return false;
  }
  return true;
}



// function to convert CAN answer to a standardized string
bool parseObdBufToPayload(const char* buf, char* outPayload, size_t outPayloadSize)
{
  if (!buf || !outPayload || outPayloadSize == 0) {
    return false;
  }

  size_t j = 0;
  bool afterColon = false;
  bool skippedLengthPrefix = false;

  for (size_t i = 0; buf[i] != '\0'; i++) {
    char c = buf[i];

    // Skip the leading length field, e.g. "02E "
    if (!skippedLengthPrefix) {
      if (c == ' ') {
        skippedLengthPrefix = true;
      }
      continue;
    }

    // When we find ":", the data section begins after it
    if (c == ':') {
      afterColon = true;
      continue;
    }

    // A line break resets the state so the next line number is ignored until the next ':'
    if (c == '\r' || c == '\n') {
      afterColon = false;
      continue;
    }

    // Before ":", ignore line numbers such as "0", "1", "2"
    if (!afterColon) {
      continue;
    }

    // Keep only hex characters from the data section itself, skipping spaces
    bool isHex =
      (c >= '0' && c <= '9') ||
      (c >= 'A' && c <= 'F') ||
      (c >= 'a' && c <= 'f');

    if (isHex) {
      if (j >= outPayloadSize - 1) {
        outPayload[0] = '\0';
        return false; // output buffer too small
      }
      // Normalize to uppercase
      if (c >= 'a' && c <= 'f') c = c - 'a' + 'A';
      outPayload[j++] = c;
    }
  }

  outPayload[j] = '\0';
  return (j > 0);
}


// Code to take a UDS DID call and send it to CAN with help of SendCANMessage
bool readUDS_DID(uint32_t canId, uint32_t did, String& outResponse)
{
  outResponse = "";
  if (!ensureAutoFlowControlEnabled()) {
    serial_log_print(LOG_INFO, "UDS read failed: auto flow control unavailable");
    return false;
  }

  uint8_t msg[4]; // the request payload for the DID call
  size_t msgLen = 0;

  for (int shift = 24; shift >= 0; shift -= 8) {
    uint8_t b = (uint8_t)((did >> shift) & 0xFF);
    if (msgLen == 0 && b == 0 && shift > 0) {
      continue;
    }
    msg[msgLen++] = b;
  }

  if (msgLen == 0) {
    serial_log_print(LOG_INFO, "UDS read failed: empty DID");
    return false;
  }

  obd.setCANID((uint16_t)canId);
  obd.setHeaderMask(0x7FF);
  obd.setHeaderFilter(canId + 0x8);  // DID reply CAN ID is normally request CAN ID + 8 (non-standard replies may differ)
  //obd.sniff();


  static char buf[1024];
  buf[0] = '\0';
  static char payload[1024];
  payload[0] = '\0';

  // Give adapter/ECU a short idle gap between consecutive UDS reads.
  delay(75);
  
  int n = obd.sendCANMessage(msg, msgLen, buf, sizeof(buf), 5000);
  if (!n) {
    serial_log_print(LOG_INFO, "UDS read failed");
    return false;
  }

  serial_log_print(LOG_INFO, "UDS raw adapter:");
  serial_log_print(LOG_INFO, buf);
  outResponse = buf;

  if (hasAdapterErrorToken(buf)) {
    serial_log_print(LOG_INFO, "UDS read failed: adapter signaled error");
    return false;
  }

  if (!parseObdBufToPayload(buf, payload, sizeof(payload))) {
    serial_log_print(LOG_INFO, "Parse failed");
    return false;
  }

  serial_log_print(LOG_INFO, "Payload only:");
  serial_log_print(LOG_INFO, payload);
  return true;
}
