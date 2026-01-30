/*************************************************************************
* Arduino Library for Freematics ONE+
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2012-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "FreematicsBase.h"
#include "FreematicsOBD.h"

// Tar bort första raden i bufferten och returnerar antal tecken som togs bort.
int dumpLine(char* buffer, int len)
{
	int bytesToDump = len >> 1;
	for (int i = 0; i < len; i++) {
		// find out first line end and discard the first line
		if (buffer[i] == '\r' || buffer[i] == '\n') {
			// go through all following \r or \n if any
			while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
			bytesToDump = i;
			break;
		}
	}
	memmove(buffer, buffer + bytesToDump, len - bytesToDump);
	return bytesToDump;
}

// Tolkar upp till fyra hextecken (med ev. mellanslag) till 16-bitarsvärde.
uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (uint8_t n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ' && n == 2) {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

// Tolkar två hextecken till 8-bitarsvärde (0 vid ogiltig input).
byte hex2uint8(const char *p)
{
	byte c1 = *p;
	byte c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >= 'a' && c1 <= 'f')
		c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 == 0)
		return (c1 & 0xf);
	else if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
		c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}

/*************************************************************************
* OBD-II UART Bridge
*************************************************************************/

// Läser ett PID-värde, normaliserar datat och skriver resultatet till result.
bool COBD::readPID(byte pid, int& result)
{
	char buffer[64];
	char* data = 0;
	sprintf(buffer, "%02X%02X\r", dataMode, pid);
	link->send(buffer);
	idleTasks();
	int ret = link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_SHORT);
	if (ret > 0 && !checkErrorMessage(buffer)) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
			p += 3;
			byte curpid = hex2uint8(p);
			if (curpid == pid) {
				errors = 0;
				while (*p && *p != ' ') p++;
				while (*p == ' ') p++;
				if (*p) {
					data = p;
					break;
				}
			}
		}
	}

	if (!data) {
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

// Läser flera PID:er och returnerar hur många som lästes korrekt.
byte COBD::readPID(const byte pid[], byte count, int result[])
{
	byte results = 0;
	for (byte n = 0; n < count; n++) {
		if (readPID(pid[n], result[n])) {
			results++;
		}
	}
	return results;
}

// Hämtar DTC-koder och fyller codes med upp till maxCodes poster.
int COBD::readDTC(uint16_t codes[], byte maxCodes)
{
	/*
	Response example:
	0: 43 04 01 08 01 09
	1: 01 11 01 15 00 00 00
	*/
	int codesRead = 0;
	if (!link) return 0;
 	for (int n = 0; n < 6; n++) {
		char buffer[128];
		sprintf(buffer, n == 0 ? "03\r" : "03%02X\r", n);
		link->send(buffer);
		if (link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) > 0) {
			if (!strstr(buffer, "NO DATA")) {
				char *p = strstr(buffer, "43");
				if (p) {
					while (codesRead < maxCodes && *p) {
						p += 6;
						if (*p == '\r') {
							p = strchr(p, ':');
							if (!p) break;
							p += 2;
						}
						uint16_t code = hex2uint16(p);
						if (code == 0) break;
						codes[codesRead++] = code;
					}
				}
				break;
			}
		}
	}
	return codesRead;
}

// Skickar kommando för att rensa ECU:s felkoder.
void COBD::clearDTC()
{
	char buffer[32];
	link->send("04\r");
	link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG);
}

// Normaliserar rådata beroende på PID:s specifikation.
int COBD::normalizeData(byte pid, char* data)
{
	int result;
	switch (pid) {
	case PID_RPM:
	case PID_EVAP_SYS_VAPOR_PRESSURE: // kPa
		result = getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE: // kPa
		result = getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = getTemperatureValue(data);
		break;
	case PID_THROTTLE:
	case PID_COMMANDED_EGR:
	case PID_COMMANDED_EVAPORATIVE_PURGE:
	case PID_FUEL_LEVEL:
	case PID_RELATIVE_THROTTLE_POS:
	case PID_ABSOLUTE_THROTTLE_POS_B:
	case PID_ABSOLUTE_THROTTLE_POS_C:
	case PID_ACC_PEDAL_POS_D:
	case PID_ACC_PEDAL_POS_E:
	case PID_ACC_PEDAL_POS_F:
	case PID_COMMANDED_THROTTLE_ACTUATOR:
	case PID_ENGINE_LOAD:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_FUEL:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getPercentageValue(data);
		break;
	case PID_MAF_FLOW: // grams/sec
		result = getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_DISTANCE_WITH_MIL: // km
	case PID_TIME_WITH_MIL: // minute
	case PID_TIME_SINCE_CODES_CLEARED: // minute
	case PID_RUNTIME: // second
	case PID_FUEL_RAIL_PRESSURE: // kPa
	case PID_ENGINE_REF_TORQUE: // Nm
		result = getLargeValue(data);
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		result = getLargeValue(data) / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		result = getLargeValue(data) / 20;
		break;
	case PID_ENGINE_TORQUE_DEMANDED: // %
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)getSmallValue(data) - 125;
		break;
	case PID_SHORT_TERM_FUEL_TRIM_1:
	case PID_LONG_TERM_FUEL_TRIM_1:
	case PID_SHORT_TERM_FUEL_TRIM_2:
	case PID_LONG_TERM_FUEL_TRIM_2:
	case PID_EGR_ERROR:
		result = ((int)getSmallValue(data) - 128) * 100 / 128;
		break;
	case PID_FUEL_INJECTION_TIMING:
		result = ((int32_t)getLargeValue(data) - 26880) / 128;
		break;
	case PID_CATALYST_TEMP_B1S1:
	case PID_CATALYST_TEMP_B2S1:
	case PID_CATALYST_TEMP_B1S2:
	case PID_CATALYST_TEMP_B2S2:
		result = getLargeValue(data) / 10 - 40;
		break;
	case PID_AIR_FUEL_EQUIV_RATIO: // 0~200
		result = (long)getLargeValue(data) * 200 / 65536;
		break;
	case PID_ODOMETER:
		if (strlen(data) < 11)
			result = -1;
		else
			result = (uint32_t)hex2uint8(data) << 24 | (uint32_t)hex2uint8(data + 3) << 16 | (uint32_t)hex2uint8(data + 6) << 8 | hex2uint8(data + 9);
		break;
	default:
		result = getSmallValue(data);
	}
	return result;
}

// Väntar på OBD-svar och returnerar pekare till datafältet i buffer.
char* COBD::getResponse(byte& pid, char* buffer, byte bufsize)
{
	if (!link) return 0;
	while (link->receive(buffer, bufsize, OBD_TIMEOUT_SHORT) > 0) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
		    p += 3;
		    byte curpid = hex2uint8(p);
		    if (pid == 0) pid = curpid;
		    if (curpid == pid) {
		        errors = 0;
		        p += 2;
		        if (*p == ' ')
		            return p + 1;
		    }
		}
	}
	return 0;
}

// Sätter adaptern i lågströmsläge via ATLP.
void COBD::enterLowPowerMode()
{
  	char buf[32];
	if (link) {
		reset();
		delay(1000);	
		link->sendCommand("ATLP\r", buf, sizeof(buf), 1000);
	}
}


// Väcker adaptern genom att skicka ett kommando tills svar fås.
void COBD::leaveLowPowerMode()
{
	// send any command to wake up
	char buf[32];
	if (!link) return;
	for (byte n = 0; n < 30 && !link->sendCommand("ATI\r", buf, sizeof(buf), 1000); n++);
}

// Returnerar pekare till första numeriska värde i svarstexten.
char* COBD::getResultValue(char* buf)
{
	char* p = buf;
	for (;;) {
		if (isdigit(*p) || *p == '-') {
			return p;
		}
		p = strchr(p, '\r');
		if (!p) break;
		if (*(++p) == '\n') p++;
	}
	return 0;
}

// Hämtar spänning från adaptern (ATRV) och returnerar värdet i volt.
float COBD::getVoltage()
{
    char buf[32];
	if (link && link->sendCommand("ATRV\r", buf, sizeof(buf), 500) > 0) {
		char* p = getResultValue(buf);
		if (p) return (float)atof(p);
    }
    return 0;
}

// Hämtar VIN och fyller buffer med ASCII-sträng vid lyckad tolkning.
bool COBD::getVIN(char* buffer, byte bufsize)
{
	for (byte n = 0; n < 2; n++) {
		if (link && link->sendCommand("0902\r", buffer, bufsize, OBD_TIMEOUT_LONG)) {
			int len = hex2uint16(buffer);
			char *p = strstr(buffer + 4, "0: 49 02 01");
			if (p) {
				char *q = buffer;
				p += 11; // skip the header
				do {
					while (*(++p) == ' ');
					for (;;) {
						*(q++) = hex2uint8(p);
						while (*p && *p != ' ') p++;
						while (*p == ' ') p++;
						if (!*p || *p == '\r') break;
					}
					p = strchr(p, ':');
				} while(p);
				*q = 0;
				if (q - buffer == len - 3) {
					return true;
				}
			}
		}
		delay(100);
	}
    return false;
}

// Kontrollerar PID mot pidmap som laddats vid init.
bool COBD::isValidPID(byte pid)
{
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return (pidmap[i] & b) != 0;
}

// Initierar OBD-adaptern, testar kommunikation och laddar PID-map.
bool COBD::init(OBD_PROTOCOLS protocol, bool quick)
{
	const char *initcmd[] = {"ATE0\r", "ATH0\r"};
	char buffer[64];
	bool success = false;

	Serial.println("[OBD:init] Step 1/7: Check link");
	if (!link) {
		Serial.println("[OBD:init] Step 1/7: FAIL (no link)");
		return false;
	}
	Serial.println("[OBD:init] Step 1/7: Link - OK");

	Serial.println("[OBD:init] Step 2/7: Set state to DISCONNECTED");
	m_state = OBD_DISCONNECTED;
	Serial.println("[OBD:init] Step 2/7: State - OK");
	Serial.println("[OBD:init] Step 3/7: Reset adapter (ATZ)");
	for (byte n = 0; n < 3; n++) {
		if (link->sendCommand("ATZ\r", buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
			Serial.print("[OBD:init] ATZ response: ");
			Serial.println(buffer);
			success = true;
			break;
		}
	}
	if (!success) {
		Serial.println("[OBD:init] Step 3/7: FAIL (ATZ)");
		return false;
	}
	Serial.println("[OBD:init] Step 3/7: ATZ - OK");
	Serial.println("[OBD:init] Step 4/7: Send init commands (ATE0/ATH0)");
	for (byte i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
		link->sendCommand(initcmd[i], buffer, sizeof(buffer), OBD_TIMEOUT_SHORT);
	}
	Serial.println("[OBD:init] Step 4/7: (ATE0/ATH0) - OK");
	if (protocol != PROTO_AUTO) {
		Serial.println("[OBD:init] Step 5/7: Set protocol (ATSP)");
		sprintf(buffer, "ATSP %X\r", protocol);
		if (!link->sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) || !strstr(buffer, "OK")) {
			Serial.println("[OBD:init] Step 5/7: FAIL (ATSP)");
			return false;
		}
		Serial.println("[OBD:init] Step 5/7: Set protocol - OK");
	}
	if (protocol == PROTO_J1939) {
		Serial.println("[OBD:init] Step 6/7: J1939 fast-path");
		m_state = OBD_CONNECTED;
		errors = 0;
		Serial.println("[OBD:init] Step 6/7: OK");
		return true;
	}

	success = false;
	Serial.println("[OBD:init] Step 6/7: Verify ECU response (PID_SPEED)");
	if (quick) {
		int value;
		if (!readPID(PID_SPEED, value)) {
			Serial.println("[OBD:init] Step 6/7: ECU FAIL (PID_SPEED)");
			return false;
		}
	} else {
		for (byte n = 0; n < 2; n++) {
			int value;
			if (readPID(PID_SPEED, value)) {
				success = true;
				break;
			}
		}
		if (!success) {
			Serial.println("[OBD:init] Step 6/7: ECU FAIL (PID_SPEED)");
			return false;
		}
	}
	Serial.println("[OBD:init] Step 6/7: ECU (PID_SPEED) - OK");

	// load pid map
	Serial.println("[OBD:init] Step 7/7: Load PID map");
	memset(pidmap, 0xff, sizeof(pidmap));
	for (byte i = 0; i < 8; i++) {
		byte pid = i * 0x20;
		sprintf(buffer, "%02X%02X\r", dataMode, pid);
		link->send(buffer);
		if (!link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) {
			break;
		}
		for (char *p = buffer; (p = strstr(p, "41 ")); ) {
			p += 3;
			if (hex2uint8(p) == pid) {
				p += 2;
				for (byte n = 0; n < 4 && *(p + n * 3) == ' '; n++) {
					pidmap[i * 4 + n] = hex2uint8(p + n * 3 + 1);
				}
				success = true;
			}
		}
	}

	if (success) {
		m_state = OBD_CONNECTED;
		errors = 0;
		Serial.println("[OBD:init] Step 7/7: OK");
	} else {
		Serial.println("[OBD:init] Step 7/7: FAIL (PID map)");
	}
	return success;
}

// Mjukåterställer adaptern (ATR).
void COBD::reset()
{
	char buf[32];
	if (link) link->sendCommand("ATR\r", buf, sizeof(buf), OBD_TIMEOUT_SHORT);
}

// Avslutar OBD-session (ATPC).
void COBD::uninit()
{
	char buf[32];
	if (link) link->sendCommand("ATPC\r", buf, sizeof(buf), OBD_TIMEOUT_SHORT);
}

// Returnerar felkod om svaret innehåller ett felmeddelande.
byte COBD::checkErrorMessage(const char* buffer)
{
	const char *errmsg[] = {"UNABLE", "ERROR", "TIMEOUT", "NO DATA"};
	for (byte i = 0; i < sizeof(errmsg) / sizeof(errmsg[0]); i++) {
		if (strstr(buffer, errmsg[i])) return i + 1;
	}
	return 0;
}

// Tolkar en byte till procentvärde 0-100.
uint8_t COBD::getPercentageValue(char* data)
{
  return (uint16_t)hex2uint8(data) * 100 / 255;
}

// Tolkar två bytes (4 hextecken) till 16-bitarsvärde.
uint16_t COBD::getLargeValue(char* data)
{
  return hex2uint16(data);
}

// Tolkar en byte (2 hextecken) till 8-bitarsvärde.
uint8_t COBD::getSmallValue(char* data)
{
  return hex2uint8(data);
}

// Tolkar temperaturbyte enligt OBD (A-40).
int16_t COBD::getTemperatureValue(char* data)
{
  return (int)hex2uint8(data) - 40;
}

// Sätter CAN header-ID och prioritet i adaptern.
void COBD::setHeaderID(uint32_t num)
{
	if (link) {
		char buf[32];
		sprintf(buf, "ATSH %X\r", num & 0xffffff);
		link->sendCommand(buf, buf, sizeof(buf), 1000);
		sprintf(buf, "ATCP %X\r", num & 0x1f);
		link->sendCommand(buf, buf, sizeof(buf), 1000);
	}
}

// Slår på/av CAN-sniffning.
void COBD::sniff(bool enabled)
{
	if (link) {
		char buf[32];
		link->sendCommand(enabled ? "ATM1\r" : "ATM0\r", buf, sizeof(buf), 1000);
	}
}

// Sätter CAN-headerfilter för sniffning.
void COBD::setHeaderFilter(uint32_t num)
{
	if (link) {
		char buf[32];
		sprintf(buf, "ATCF %X\r", num);
		link->sendCommand(buf, buf, sizeof(buf), 1000);
	}
}
	
// Sätter CAN-headermask för sniffning.
void COBD::setHeaderMask(uint32_t bitmask)
{
	if (link) {
		char buf[32];
		sprintf(buf, "ATCM %X\r", bitmask);
		link->sendCommand(buf, buf, sizeof(buf), 1000);
	}
}

// Tar emot sniffade CAN-rader och avkodar till rå bytebuffert.
int COBD::receiveData(byte* buf, int len)
{
	if (!link) return 0;
	int n = 0;
	for (n = 0; n < len; ) {
		int c = link->read();
		if (c == -1 || c == '\r') break;
		buf[n++] = c;
	}
	if (n == 0) return 0;
	int bytes = 0;
	len = n;
	if (buf[0] == '$') {
		for (n = 1; n < len && buf[n] != ','; n++);
		for (; n < len && buf[n] == ','; bytes++) {
			byte d = hex2uint8((const char*)buf + n + 1);
			n += 3;
			if (buf[n] != ',' && buf[n] != '\r') {
				if (d != hex2uint8((const char*)buf + n)) break;
				n += 2;
			}
			buf[bytes] = d;
		}
	} else {
		for (n = 0; n < len; bytes++) {
			buf[bytes] = hex2uint8((const char*)buf + n);
			n += 2;
			if (buf[n++] != ' ') break;
		}
	}
	return bytes;
}

// Sätter CAN-ID för sändning av kommande ramar.
void COBD::setCANID(uint16_t id)
{
	if (link) {
		char buf[32];
		sprintf(buf, "ATSH %X\r", id);
		link->sendCommand(buf, buf, sizeof(buf), 1000);
	}
}

// Skickar CAN-data som hexsträng och returnerar antal svarstecken.
int COBD::sendCANMessage(byte msg[], int len, char* buf, int bufsize)
{
	if (!link) return 0;
	char cmd[258];
	if (len * 2 >= sizeof(cmd) - 1) len = sizeof(cmd) / 2 - 2; 
	for (int n = 0; n < len; n++) {
		sprintf(cmd + n * 2, "%02X", msg[n]); 
	}
	cmd[len * 2] = '\r';
	cmd[len * 2 + 1] = 0;
	return link->sendCommand(cmd, buf, bufsize, 100);
}
