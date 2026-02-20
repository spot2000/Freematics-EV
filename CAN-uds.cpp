/* CAN UDS (Unified Diagnostic Services) integration */
/* Below is functions to read and interpret UDS messages from CAN bus */
/* it uses functions from FreematicsOBD.cpp */
/* Functions to add: read UDS messages, interpret UDS responses */

#include <FreematicsPlus.h>
#include "CAN-uds.h"


// Add your UDS related functions here


extern COBD obd;  // eller globalt: COBD obd;

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

// "220105" eller "22 01 05" -> bytes[]
// Returnerar antal bytes, 0 vid fel.
static size_t hexStringToBytes(const char* s, uint8_t* out, size_t outMax) {
  if (!s || !out || outMax == 0) return 0;

  size_t n = 0;
  int hi = -1;

  while (*s && n < outMax) {
    char c = *s++;

    // hoppa whitespace och separators
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == ':' || c == '-') {
      continue;
    }

    int v = hexNibble(c);
    if (v < 0) {
      // ogiltigt tecken
      return 0;
    }

    if (hi < 0) {
      hi = v; // första nibble
    } else {
      out[n++] = (uint8_t)((hi << 4) | v);
      hi = -1;
    }
  }

  // udda antal hex-tecken => fel
  if (hi >= 0) return 0;

  return n;
}

// Parsar ASCII-hex i buf (t.ex. "62 01 05 12 34") till bytes
static size_t parseHexBytes(const char* s, uint8_t* out, size_t outMax) {
  // samma logik duger här också
  return hexStringToBytes(s, out, outMax);
}

// Försöker extrahera en data-rad ur adaptertext (echo/prompt/headers förekommer ofta).
static size_t parseAdapterResponse(const char* text, uint8_t* out, size_t outMax)
{
  if (!text || !out || !outMax) return 0;

  const char* p = text;
  while (*p) {
    while (*p == '\r' || *p == '\n' || *p == ' ') p++;
    if (!*p) break;

    const char* lineStart = p;
    while (*p && *p != '\r' && *p != '\n') p++;
    const char* lineEnd = p;

    while (lineStart < lineEnd && *lineStart == ' ') lineStart++;
    while (lineEnd > lineStart && (lineEnd[-1] == ' ' || lineEnd[-1] == '\t')) lineEnd--;
    if (lineEnd <= lineStart) continue;

    size_t lineLen = (size_t)(lineEnd - lineStart);
    if (lineLen >= 2 && lineLen < 128) {
      char line[128];
      memcpy(line, lineStart, lineLen);
      line[lineLen] = '\0';

      // Rader som "2: FF 00 ...": hoppa radindex-prefix.
      const char* payload = line;
      int prefixDigits = 0;
      while (payload[prefixDigits] && payload[prefixDigits] >= '0' && payload[prefixDigits] <= '9') {
        prefixDigits++;
      }
      if (prefixDigits > 0 && payload[prefixDigits] == ':') {
        payload += prefixDigits + 1;
        while (*payload == ' ') payload++;
      }

      // Rader som "7EC 06 62 01 05 ...": hoppa CAN-ID + längdbyte.
      int firstTokenLen = 0;
      while (payload[firstTokenLen] && payload[firstTokenLen] != ' ') firstTokenLen++;
      if ((firstTokenLen == 3 || firstTokenLen == 8) && payload[firstTokenLen] == ' ') {
        bool tokenHex = true;
        for (int i = 0; i < firstTokenLen; i++) {
          if (hexNibble(payload[i]) < 0) {
            tokenHex = false;
            break;
          }
        }
        if (tokenHex) {
          const char* rest = payload + firstTokenLen + 1;
          if (rest[0] && rest[1] && hexNibble(rest[0]) >= 0 && hexNibble(rest[1]) >= 0) {
            rest += 2;
            while (*rest == ' ') rest++;
          }
          size_t n = parseHexBytes(rest, out, outMax);
          if (n) return n;
        }
      }

      size_t n = parseHexBytes(payload, out, outMax);
      if (n) return n;
    }
  }
  return 0;
}

/**
 * read_UDS
 *  txCanId: t.ex 0x7E4
 *  udsRequestHex: t.ex "220105" eller "22 01 05"
 *
 *  outRespTxt: råsvaret som ASCII-hex från Freematics
 *  outRespBytes/outRespLen: samma svar som bytes
 *
 * Returnerar true vid svar, annars false.
 */
bool read_UDS(uint32_t txCanId,
              const char* udsRequestHex,
              char* outRespTxt, size_t outRespTxtSize,
              uint8_t* outRespBytes, size_t outRespBytesMax, size_t* outRespLen)
{
  if (!udsRequestHex || !outRespTxt || outRespTxtSize < 4 || !outRespBytes || !outRespLen) {
    return false;
  }

  // 1) Konvertera request-hex -> request-bytes
  uint8_t req[32];  // UDS single-frame payload brukar vara kort; öka om du vill
  size_t reqLen = hexStringToBytes(udsRequestHex, req, sizeof(req));
  if (reqLen == 0) {
    *outRespLen = 0;
    outRespTxt[0] = '\0';
    return false;
  }

  const uint32_t rxCanId = txCanId + 0x8; // enligt din bekräftelse

  // 2) Sätt TX-ID
  obd.setCANID(txCanId);

  // 3) Sätt RX filter
  //    Tillåt svar från närliggande ECU-ID (t.ex. 0x7E8-0x7EF) genom
  //    att maska bort de 3 lägsta bitarna. Detta gör läsningen mer robust
  //    när ECU svarar på ett annat ID än exakt tx+0x8.
  const uint32_t rxMask = 0x7F8;
  obd.setHeaderMask(rxMask);
  obd.setHeaderFilter(rxCanId & rxMask);

  // 4) Skicka request
  outRespTxt[0] = '\0';
  *outRespLen = 0;

  auto sendAndTryParse = [&](const uint8_t* data, size_t len) -> bool {
    char sendBuf[64];
    int sendResp = obd.sendCANMessage((byte*)data, (byte)len, sendBuf, (int)sizeof(sendBuf));
    if (sendResp <= 0) {
      return false;
    }

    // Vissa adaptrar returnerar svaret direkt i sendCommand-buffern (utan sniffning).
    size_t parsed = parseAdapterResponse(sendBuf, outRespBytes, outRespBytesMax);
    if (parsed > 0) {
      *outRespLen = parsed;
    }
    return true;
  };

  // Standardfall: skicka payload som "22 01 05" och låt adaptern formatera ISO-TP.
  bool sent = sendAndTryParse(req, reqLen);

  // Fallback: vissa adaptrar/inställningar kräver explicit Single-Frame PCI-byte.
  // Exempel: 03 22 01 05.
  if (!sent || *outRespLen == 0) {
    if (reqLen <= 7) {
      uint8_t sf[8];
      sf[0] = (uint8_t)reqLen;
      memcpy(sf + 1, req, reqLen);

      // Testa först minimal SF-längd (1 + reqLen).
      sent = sendAndTryParse(sf, reqLen + 1);

      // Extra fallback: testa full 8-byte frame med nollpadding.
      if ((!sent || *outRespLen == 0) && reqLen + 1 < sizeof(sf)) {
        memset(sf + 1 + reqLen, 0, sizeof(sf) - (1 + reqLen));
        sent = sendAndTryParse(sf, sizeof(sf));
      }
    }
  }

  if (!sent) {
    return false;
  }

  // 5) Läs ISO-TP svar (single- och multi-frame)
  bool sniffEnabled = false;
  if (*outRespLen == 0) {
    obd.sniff(true);
    sniffEnabled = true;
  }
  uint32_t startMs = millis();
  uint32_t lastMs = startMs;
  size_t expectedLen = 0;
  uint8_t nextSeq = 1;
  bool gotFrame = (*outRespLen > 0);

  while (millis() - startMs < 1000) {
    byte frame[16];
    int n = obd.receiveData(frame, sizeof(frame));
    if (n <= 0) {
      delay(2);
      continue;
    }
    gotFrame = true;
    lastMs = millis();

    uint8_t pci = frame[0];
    uint8_t type = pci >> 4;

    if (type == 0x0) { // Single Frame
      uint8_t len = pci & 0x0F;
      size_t copyLen = (len <= (uint8_t)(n - 1)) ? len : (size_t)(n - 1);
      if (copyLen > outRespBytesMax) copyLen = outRespBytesMax;
      memcpy(outRespBytes, frame + 1, copyLen);
      *outRespLen = copyLen;
      expectedLen = copyLen;
      break;
    } else if (type == 0x1) { // First Frame
      expectedLen = ((size_t)(pci & 0x0F) << 8) | frame[1];
      size_t copyLen = (n > 2) ? (size_t)(n - 2) : 0;
      if (copyLen > outRespBytesMax) copyLen = outRespBytesMax;
      if (copyLen > expectedLen) copyLen = expectedLen;
      memcpy(outRespBytes, frame + 2, copyLen);
      *outRespLen = copyLen;

      // Skicka Flow Control (Continue To Send)
      uint8_t fc[3] = {0x30, 0x00, 0x00};
      obd.setCANID(txCanId);
      char fcResp[16];
      obd.sendCANMessage(fc, 3, fcResp, (int)sizeof(fcResp));

      nextSeq = 1;
    } else if (type == 0x2) { // Consecutive Frame
      if (expectedLen == 0) {
        continue;
      }
      uint8_t seq = pci & 0x0F;
      if ((seq & 0x0F) != (nextSeq & 0x0F)) {
        nextSeq = seq;
      }
      size_t remaining = expectedLen - *outRespLen;
      size_t copyLen = (n > 1) ? (size_t)(n - 1) : 0;
      if (copyLen > remaining) copyLen = remaining;
      if (*outRespLen + copyLen > outRespBytesMax) {
        copyLen = outRespBytesMax - *outRespLen;
      }
      if (copyLen > 0) {
        memcpy(outRespBytes + *outRespLen, frame + 1, copyLen);
        *outRespLen += copyLen;
      }
      nextSeq++;
      if (*outRespLen >= expectedLen) {
        break;
      }
    }

    if (expectedLen > 0 && *outRespLen >= expectedLen) {
      break;
    }

    if (gotFrame && expectedLen > 0 && millis() - lastMs > 200) {
      break;
    }
  }
  if (sniffEnabled) {
    obd.sniff(false);
  }

  if (!gotFrame || *outRespLen == 0) {
    return false;
  }

  // 6) Bygg ASCII-hex för komplett payload
  size_t pos = 0;
  for (size_t i = 0; i < *outRespLen && pos + 3 < outRespTxtSize; i++) {
    snprintf(outRespTxt + pos, outRespTxtSize - pos, "%02X", outRespBytes[i]);
    pos += 2;
    if (pos + 2 < outRespTxtSize && i + 1 < *outRespLen) {
      outRespTxt[pos++] = ' ';
      outRespTxt[pos] = '\0';
    }
  }
  outRespTxt[pos] = '\0';
  return true;
}


void UDS_read_test() {
  char respTxt[256];
  uint8_t resp[128];
  size_t respLen = 0;

  if (read_UDS(0x7E4, "220105", respTxt, sizeof(respTxt), resp, sizeof(resp), &respLen)) {
    Serial.print("RAW: ");
    Serial.println(respTxt);

    // snabbkoll: positivt svar börjar med 62 01 05
    if (respLen >= 3 && resp[0] == 0x62 && resp[1] == 0x01 && resp[2] == 0x05) {
      // data börjar från resp[3]
    }
  } else {
    Serial.println("No response / request parse error");
  }
}
