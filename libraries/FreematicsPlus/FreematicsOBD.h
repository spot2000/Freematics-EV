/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#ifndef FREEMATICS_OBD
#define FREEMATICS_OBD

#include "utility/OBD.h"

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */

/**
 * @brief Removes the first response line from the buffer.
 *
 * The remaining content is shifted to the beginning of the same buffer.
 *
 * @param buffer Mutable response buffer to process in-place.
 * @param len Number of valid characters currently stored in @p buffer.
 * @return Number of characters removed from the beginning of @p buffer.
 */
int dumpLine(char* buffer, int len);
/**
 * @brief Converts up to four hexadecimal characters into a 16-bit unsigned integer.
 * @param p Pointer to a character sequence that starts with hexadecimal data.
 * @return Parsed 16-bit value.
 */
uint16_t hex2uint16(const char *p);
/**
 * @brief Converts one or two hexadecimal characters into an 8-bit unsigned integer.
 * @param p Pointer to a character sequence that starts with hexadecimal data.
 * @return Parsed 8-bit value, or 0 for invalid input.
 */
byte hex2uint8(const char *p);

/**
 * @brief OBD-II high-level interface for Freematics serial adapters.
 *
 * This class wraps AT commands and OBD service requests to:
 * - initialize and close OBD sessions,
 * - read and normalize PID values,
 * - fetch and clear DTC codes,
 * - perform VIN/voltage queries,
 * - and send/receive raw CAN traffic.
 */
class COBD
{
public:
	/**
	 * @brief Binds the communication transport implementation.
	 * @param link Pointer to a CLink instance used for physical I/O.
	 */
	void begin(CLink* link) { this->link = link; }
	/**
	 * @brief Initializes adapter and ECU communication.
	 * @param protocol Requested OBD protocol (auto-detect by default).
	 * @param quick If true, performs a shorter initialization path.
	 * @return true if initialization succeeds; otherwise false.
	 */
	bool init(OBD_PROTOCOLS protocol = PROTO_AUTO, bool quick = false);
	/**
	 * @brief Performs a soft reset of the OBD adapter.
	 */
	void reset();
	/**
	 * @brief Gracefully closes the OBD session.
	 */
	void uninit();
	/**
	 * @brief Sets serial baud rate for the underlying link.
	 * @param baudrate Requested line speed in bits per second.
	 * @return true if the baud rate was applied; otherwise false.
	 */
	bool setBaudRate(unsigned long baudrate);
	/**
	 * @brief Returns current connection state.
	 * @return Current OBD state enum value.
	 */
	OBD_STATES getState() { return m_state; }
	/**
	 * @brief Reads one OBD-II PID and normalizes its payload.
	 * @param pid PID identifier to query.
	 * @param result Output reference receiving normalized integer value.
	 * @return true on successful read/parse; otherwise false.
	 */
	bool readPID(byte pid, int& result);
	/**
	 * @brief Reads several PIDs in sequence.
	 * @param pid Array of PID identifiers to query.
	 * @param count Number of elements in @p pid and @p result.
	 * @param result Output array receiving normalized values per PID.
	 * @return Number of PIDs successfully read.
	 */
	byte readPID(const byte pid[], byte count, int result[]);
	/**
	 * @brief Requests adapter low-power mode.
	 */
	void enterLowPowerMode();
	/**
	 * @brief Wakes adapter from low-power mode.
	 */
	void leaveLowPowerMode();
	/**
	 * @brief Reads diagnostic trouble codes (DTC) from ECU.
	 * @param codes Output buffer receiving raw DTC values.
	 * @param maxCodes Maximum number of entries to write into @p codes.
	 * @return Number of DTC entries written.
	 */
	int readDTC(uint16_t codes[], byte maxCodes = 1);
	/**
	 * @brief Clears stored DTC codes in ECU.
	 */
	void clearDTC();
	/**
	 * @brief Reads adapter-reported supply voltage using ATRV.
	 * @return Voltage in volts, or 0 when unavailable.
	 */
	float getVoltage();
	/**
	 * @brief Reads and decodes vehicle VIN to ASCII.
	 * @param buffer Output character buffer for VIN string (NUL-terminated on success).
	 * @param bufsize Size of @p buffer in bytes.
	 * @return true if VIN was decoded successfully; otherwise false.
	 */
	bool getVIN(char* buffer, byte bufsize);
	/**
	 * @brief Checks whether a PID is marked as supported in the local PID map.
	 * @param pid PID number to test.
	 * @return true if supported; otherwise false.
	 */
	bool isValidPID(byte pid);
	/**
	 * @brief Sets custom CAN transmit header and priority.
	 * @param num Header value used by ATSH/ATCP adapter commands.
	 */
	void setHeaderID(uint32_t num);
	/**
	 * @brief Enables or disables adapter monitor/sniff mode.
	 * @param enabled true to enable monitoring; false to disable.
	 */
	void sniff(bool enabled = true);
	/**
	 * @brief Sets CAN header filter for sniffed traffic.
	 * @param num Filter value passed to ATCF.
	 */
	void setHeaderFilter(uint32_t num);
	/**
	 * @brief Sets CAN header mask for sniffing filter logic.
	 * @param bitmask Mask value passed to ATCM.
	 */
	void setHeaderMask(uint32_t bitmask);
	/**
	 * @brief Receives one monitor line and decodes it into raw bytes.
	 * @param buf Input/output buffer used for line capture and decoded bytes.
	 * @param len Maximum number of bytes to decode into @p buf.
	 * @return Number of decoded payload bytes.
	 */
	int receiveData(byte* buf, int len);
	/**
	 * @brief Sets CAN identifier for outgoing frames.
	 * @param id 11-bit or 29-bit CAN identifier, depending on adapter mode.
	 */
	void setCANID(uint16_t id);
	/**
	 * @brief Sends one CAN message encoded as hexadecimal characters.
	 * @param msg Input byte array with payload to transmit.
	 * @param len Number of payload bytes in @p msg.
	 * @param buf Output buffer receiving adapter response text.
	 * @param bufsize Capacity of @p buf.
	 * @param timeout Command timeout in milliseconds.
	 * @return Number of characters received in adapter response, or 0 on failure.
	 */
	int sendCANMessage(byte msg[], int len, char* buf, int bufsize, unsigned int timeout = 100);
	// set current PID mode
	byte dataMode = 1;
	// occurrence of errors
	byte errors = 0;
	// bit map of supported PIDs
	byte pidmap[4 * 8] = {0};
	// link object pointer
	CLink* link = 0;
protected:
	/**
	 * @brief Idle hook called while waiting for adapter responses.
	 */
	virtual void idleTasks() { delay(5); }
	/**
	 * @brief Reads raw adapter response and returns pointer to PID data field.
	 * @param pid Input/output PID selector (0 means accept first PID found).
	 * @param buffer Output buffer for adapter response text.
	 * @param bufsize Size of @p buffer.
	 * @return Pointer to parsed data field inside @p buffer, or nullptr on failure.
	 */
	char* getResponse(byte& pid, char* buffer, byte bufsize);
	/**
	 * @brief Converts one-byte PID payload into 0..100% scale.
	 * @param data Pointer to ASCII hexadecimal payload.
	 * @return Percentage value in integer form.
	 */
	uint8_t getPercentageValue(char* data);
	/**
	 * @brief Converts two-byte ASCII hexadecimal payload into 16-bit integer.
	 * @param data Pointer to ASCII hexadecimal payload.
	 * @return Parsed unsigned 16-bit value.
	 */
	uint16_t getLargeValue(char* data);
	/**
	 * @brief Converts one-byte ASCII hexadecimal payload into 8-bit integer.
	 * @param data Pointer to ASCII hexadecimal payload.
	 * @return Parsed unsigned 8-bit value.
	 */
	uint8_t getSmallValue(char* data);
	/**
	 * @brief Converts temperature payload according to OBD formula (A-40).
	 * @param data Pointer to ASCII hexadecimal payload.
	 * @return Temperature in degrees Celsius.
	 */
	int16_t getTemperatureValue(char* data);
	/**
	 * @brief Applies PID-specific conversion from raw payload to engineering value.
	 * @param pid PID that defines conversion rule.
	 * @param data Pointer to ASCII hexadecimal payload.
	 * @return Normalized integer value.
	 */
	int normalizeData(byte pid, char* data);
	/**
	 * @brief Detects common adapter error text in a response string.
	 * @param buffer Adapter response text.
	 * @return 0 when no error string is found; otherwise non-zero error index.
	 */
	byte checkErrorMessage(const char* buffer);
	/**
	 * @brief Finds first numeric token in a multi-line response.
	 * @param buf Adapter response text to scan.
	 * @return Pointer to first numeric token within @p buf, or nullptr if none.
	 */
	char* getResultValue(char* buf);
	OBD_STATES m_state = OBD_DISCONNECTED;
};

#endif
