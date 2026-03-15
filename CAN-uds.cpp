/* CAN UDS (Unified Diagnostic Services) integration */
/* Below is functions to read and interpret UDS messages from CAN bus */
/* it uses functions from FreematicsOBD.cpp */
/* Functions to add: read UDS messages, interpret UDS responses */

#include <FreematicsPlus.h>
#include "CAN-uds.h"


// Add your UDS related functions here


extern COBD obd;  // eller globalt: COBD obd;

// Cacha senast använda TX-CAN-ID för att undvika onödiga ATSH/setCANID-anrop.
static uint32_t g_prevUdsTxCanId = 0xFFFFFFFF;

static bool isExpectedUdsReply(const uint8_t* data, size_t len,
                               const uint8_t* req, size_t reqLen);

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

static bool isHexToken(const char* token, int len)
{
  if (!token || len <= 0) return false;
  for (int i = 0; i < len; i++) {
    if (hexNibble(token[i]) < 0) return false;
  }
  return true;
}

// Extraherar payload-bytes ur en enskild adapterrad.
// Stödjer rader som:
//  "7EC 10 2E 62 ..."
//  "0: 10 2E 62 ..."
//  "10 2E 62 ..."
static size_t parseAdapterLinePayload(const char* line, uint8_t* out, size_t outMax)
{
  if (!line || !out || !outMax) return 0;

  const char* payload = line;
  while (*payload == ' ' || *payload == '\t') payload++;

  // Hoppa över index-prefix "N:".
  int idxDigits = 0;
  while (payload[idxDigits] >= '0' && payload[idxDigits] <= '9') idxDigits++;
  if (idxDigits > 0 && payload[idxDigits] == ':') {
    payload += idxDigits + 1;
    while (*payload == ' ') payload++;
  }

  // Hoppa över CAN-ID prefix (3 eller 8 hextecken) men inte första databyten.
  // Stödjer även varianter som "0x7EC" och/eller ':' separator.
  if (payload[0] == '0' && (payload[1] == 'x' || payload[1] == 'X')) {
    payload += 2;
  }

  int tokenLen = 0;
  while (payload[tokenLen] && payload[tokenLen] != ' ' && payload[tokenLen] != ':') tokenLen++;
  if ((tokenLen == 3 || tokenLen == 8) && isHexToken(payload, tokenLen)) {
    if (payload[tokenLen] == ':') {
      payload += tokenLen + 1;
      while (*payload == ' ') payload++;
    } else if (payload[tokenLen] == ' ') {
      payload += tokenLen + 1;
      while (*payload == ' ') payload++;
    }
  }

  size_t parsed = parseHexBytes(payload, out, outMax);
  if (parsed) return parsed;

  // Fallback: vissa adaptrar/loggformat kan lägga till extra kolumner före payload.
  // Skanna hela raden efter en möjlig PCI-byte (0x0*,0x1*,0x2*,0x3*) och
  // försök därefter tolka resten som hex-bytes.
  const char* p = line;
  while (*p) {
    while (*p == ' ' || *p == '\t' || *p == ':' || *p == '|') p++;
    if (!*p) break;

    const char* tok = p;
    int tokLen = 0;
    while (p[tokLen] && p[tokLen] != ' ' && p[tokLen] != '\t' && p[tokLen] != ':' && p[tokLen] != '|') tokLen++;

    if (tokLen == 2 && isHexToken(tok, tokLen)) {
      int b = (hexNibble(tok[0]) << 4) | hexNibble(tok[1]);
      int pciType = (b >> 4) & 0x0F;
      if (pciType <= 0x3) {
        parsed = parseHexBytes(tok, out, outMax);
        if (parsed) return parsed;
      }
    }
    p += tokLen;
  }

  return 0;
}

static size_t collectIsoTpPayload(const char* text,
                                  uint8_t* out, size_t outMax,
                                  const uint8_t* req, size_t reqLen)
{
  if (!text || !out || !outMax || !req || !reqLen) return 0;

  uint8_t ffBuffer[512];
  size_t ffWritten = 0;
  size_t ffExpected = 0;
  bool gotFirstFrame = false;
  uint8_t expectedSeq = 1;

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
    if (lineLen >= 127) continue;

    char line[128];
    memcpy(line, lineStart, lineLen);
    line[lineLen] = '\0';

    uint8_t frame[64];
    size_t frameLen = parseAdapterLinePayload(line, frame, sizeof(frame));
    if (!frameLen) continue;

    uint8_t pciType = frame[0] >> 4;

    if (pciType == 0x0) {
      size_t sfLen = frame[0] & 0x0F;
      if (sfLen > frameLen - 1) sfLen = frameLen - 1;
      if (sfLen == 0 || sfLen > outMax) continue;
      memcpy(out, frame + 1, sfLen);
      if (isExpectedUdsReply(out, sfLen, req, reqLen)) {
        return sfLen;
      }
      continue;
    }

    if (pciType == 0x1) {
      if (frameLen < 2) continue;
      ffExpected = ((size_t)(frame[0] & 0x0F) << 8) | frame[1];
      ffWritten = 0;
      gotFirstFrame = false;
      expectedSeq = 1;
      if (frameLen > 2) {
        size_t copyLen = frameLen - 2;
        if (copyLen > sizeof(ffBuffer)) copyLen = sizeof(ffBuffer);
        memcpy(ffBuffer, frame + 2, copyLen);
        ffWritten = copyLen;
      }
      if (ffExpected >= 3 && ffWritten >= 3 && isExpectedUdsReply(ffBuffer, ffWritten, req, reqLen)) {
        gotFirstFrame = true;
      }
      continue;
    }

    if (pciType == 0x2 && gotFirstFrame && ffExpected > ffWritten) {
      uint8_t seq = frame[0] & 0x0F;
      if (seq != expectedSeq) {
        expectedSeq = (uint8_t)((seq + 1) & 0x0F);
      } else {
        expectedSeq = (uint8_t)((expectedSeq + 1) & 0x0F);
      }

      if (frameLen > 1) {
        size_t copyLen = frameLen - 1;
        size_t remain = ffExpected - ffWritten;
        if (copyLen > remain) copyLen = remain;
        if (copyLen > sizeof(ffBuffer) - ffWritten) copyLen = sizeof(ffBuffer) - ffWritten;
        memcpy(ffBuffer + ffWritten, frame + 1, copyLen);
        ffWritten += copyLen;
      }

      if (ffWritten >= ffExpected) {
        size_t outLen = ffExpected;
        if (outLen > outMax) outLen = outMax;
        memcpy(out, ffBuffer, outLen);
        return outLen;
      }
    }
  }

  return 0;
}

static void printIndexedAdapterFrames(const char* text)
{
  if (!text) return;
  const char* p = text;
  int frameIdx = 0;
  while (*p) {
    while (*p == '\r' || *p == '\n') p++;
    if (!*p) break;

    const char* lineStart = p;
    while (*p && *p != '\r' && *p != '\n') p++;
    const char* lineEnd = p;

    while (lineStart < lineEnd && (*lineStart == ' ' || *lineStart == '\t')) lineStart++;
    if (lineEnd <= lineStart) continue;

    size_t lineLen = (size_t)(lineEnd - lineStart);
    if (lineLen >= 127) continue;

    char line[128];
    memcpy(line, lineStart, lineLen);
    line[lineLen] = '\0';

    uint8_t frame[64];
    size_t frameLen = parseAdapterLinePayload(line, frame, sizeof(frame));
    if (!frameLen) continue;

    Serial.print("[UDS] RX IDX ");
    Serial.print(frameIdx++);
    Serial.print(": ");
    for (size_t i = 0; i < frameLen; i++) {
      if (i) Serial.print(' ');
      if (frame[i] < 16) Serial.print('0');
      Serial.print(frame[i], HEX);
    }
    Serial.println();
  }
}

static void printRawAdapterResponse(const char* text, int textLen)
{
  Serial.print("[UDS] RX RAW (len=");
  Serial.print(textLen);
  Serial.println("): ");

  if (!text || !*text) {
    Serial.println("[UDS] RX RAW <empty>");
    return;
  }

  Serial.println(text);
}

static bool isExpectedUdsReply(const uint8_t* data, size_t len,
                               const uint8_t* req, size_t reqLen)
{
  if (!data || !len || !req || !reqLen) return false;

  // Negativt svar för just vår service (7F <SID> <NRC>)
  if (len >= 3 && data[0] == 0x7F) {
    return data[1] == req[0];
  }

  const uint8_t positiveSid = (uint8_t)(req[0] + 0x40);
  if (data[0] != positiveSid) return false;

  // För tjänster som brukar echa parametrar (t.ex. 22 DID_H DID_L)
  // verifierar vi upp till två bytes extra för att filtrera bort skräpramar.
  size_t echoBytes = (reqLen > 1) ? reqLen - 1 : 0;
  if (echoBytes > 2) echoBytes = 2;
  if (len < 1 + echoBytes) return false;
  for (size_t i = 0; i < echoBytes; i++) {
    if (data[1 + i] != req[1 + i]) return false;
  }
  return true;
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

  // 2) Sätt TX-ID
  obd.setCANID(txCanId);

  // 3) Skicka request i normal adapter-mode.
  //    Skicka endast UDS-payload (t.ex. 22 01 05); adaptern hanterar längdfältet.
  outRespTxt[0] = '\0';
  *outRespLen = 0;

  auto sendAndTryParse = [&](const uint8_t* data, size_t len) -> bool {
    char sendBuf[1024];
    memset(sendBuf, 0, sizeof(sendBuf));
    int sendResp = obd.sendCANMessage((byte*)data, (byte)len, sendBuf, (int)sizeof(sendBuf), 900);

    bool gotText = sendBuf[0] != '\0';
    if (sendResp <= 0 && !gotText) return false;

    size_t parsed = collectIsoTpPayload(sendBuf, outRespBytes, outRespBytesMax, req, reqLen);
    if (parsed > 0) {
      *outRespLen = parsed;
    }
    return sendResp > 0 || gotText;
  };

  bool sent = false;
  for (int attempt = 0; attempt < 3 && !*outRespLen; attempt++) {
    sent = sendAndTryParse(req, reqLen) || sent;
    if (!*outRespLen) delay(30);
  }

  if (!sent) {
    return false;
  }

  if (*outRespLen > 0) {
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

  return false;
}


String UDS_read_DID(const char* canIdHex, const char* didHex) {
  // TX/CAN-ID och DID kan skickas som hexsträngar, t.ex. "7E4" och "220101".
  uint8_t didReq[32];
  size_t didReqLen = hexStringToBytes(didHex, didReq, sizeof(didReq));
  if (!canIdHex || !didReqLen) {
    return String();
  }

  uint32_t txCanId = (uint32_t)strtoul(canIdHex, nullptr, 16);

  // Lyssna på ECU-svar från txCanId + 0x8 (normal 11-bit addressing).
  // Sätt endast CAN-ID när det ändrats sedan föregående UDS_read_DID-anrop.
  if (g_prevUdsTxCanId != txCanId) {
    obd.setCANID(txCanId);
    g_prevUdsTxCanId = txCanId;
  }
  obd.setHeaderMask(0xFFFFFF);
  obd.setHeaderFilter(txCanId + 0x8);

  byte msg[32];
  memcpy(msg, didReq, didReqLen);
  char buf[1024];
  memset(buf, 0, sizeof(buf));

  Serial.print("[UDS] TX ");
  Serial.print(canIdHex);
  Serial.print(": ");
  Serial.print(didHex);
  Serial.print(" (filter ");
  Serial.print(txCanId + 0x8, HEX);
  Serial.println(")");

  bool gotRaw = false;
  String DIDanswer = "62";
  for (int attempt = 0; attempt < 3 && !gotRaw; attempt++) {
    DIDanswer = "62";
    memset(buf, 0, sizeof(buf));
    int rxLen = obd.sendCANMessage(msg, didReqLen, buf, sizeof(buf), 900);

    // Visa exakt rå adaptertext utan tolkning/parsning.
    printRawAdapterResponse(buf, rxLen);
    printIndexedAdapterFrames(buf);

    const char* pFrames = buf;
    while (*pFrames) {
      while (*pFrames == '\r' || *pFrames == '\n') pFrames++;
      if (!*pFrames) break;

      const char* lineStart = pFrames;
      while (*pFrames && *pFrames != '\r' && *pFrames != '\n') pFrames++;
      const char* lineEnd = pFrames;

      while (lineStart < lineEnd && (*lineStart == ' ' || *lineStart == '\t')) lineStart++;
      while (lineEnd > lineStart && (lineEnd[-1] == ' ' || lineEnd[-1] == '\t')) lineEnd--;
      if (lineEnd <= lineStart) continue;

      size_t lineLen = (size_t)(lineEnd - lineStart);
      if (lineLen >= 127) continue;

      char line[128];
      memcpy(line, lineStart, lineLen);
      line[lineLen] = '\0';

      uint8_t frame[64];
      size_t frameLen = parseAdapterLinePayload(line, frame, sizeof(frame));
      if (!frameLen) continue;

      for (size_t i = 0; i < frameLen; i++) {
        char hex[3];
        snprintf(hex, sizeof(hex), "%02X", frame[i]);
        DIDanswer += hex;
      }
    }

    //Serial.print("DID svar är: ");
    //Serial.println(DIDanswer);

    if (rxLen <= 0 && !buf[0]) {
      Serial.println("[UDS] RX timeout/no buffered data, retrying...");
      delay(30);
      continue;
    }

    uint8_t udsBytes[128];
    size_t udsLen = collectIsoTpPayload(buf, udsBytes, sizeof(udsBytes), msg, didReqLen);
    if (udsLen) {
      Serial.print("[UDS] RX PARSED ");
      for (size_t i = 0; i < udsLen; i++) {
        if (i) Serial.print(' ');
        if (udsBytes[i] < 16) Serial.print('0');
        Serial.print(udsBytes[i], HEX);
      }
      Serial.println();
    }

    // Markera lyckat om vi fick någon faktisk text tillbaka (inte bara tomrad/OK-echo).
    const char* p = buf;
    while (*p == ' ' || *p == '\r' || *p == '\n') p++;
    if (*p && strcmp(p, "OK") != 0) {
      gotRaw = true;
    } else {
      Serial.println("[UDS] RX only adapter echo/empty, retrying...");
      delay(30);
    }
  }

  if (!gotRaw) {
    Serial.println("[UDS] No raw UDS response after retries");
  }

  return DIDanswer;
}
