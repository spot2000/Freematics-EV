/**
 * @file FreematicsBase.h
 * @brief Base class and data structures for Freematics telematics products
 * 
 * Defines the core interfaces and data types used across Freematics telematics devices.
 * Includes custom PID definitions for GPS, sensor data, and communication protocols.
 * 
 * @author Stanley Huang <stanley@freematics.com.au>
 * @copyright (C)2017-2018
 * @license BSD
 * 
 * Visit https://freematics.com for more information
 */

/**
 * @defgroup CustomPIDs Custom Parameter IDs
 * @brief Non-OBD and custom PID definitions for extended sensor data
 * @{
 */

/** @defgroup GPSPIDs GPS-related PIDs */
/** @defgroup SensorPIDs Sensor-related PIDs */
/** @defgroup SystemPIDs System-related PIDs */

/** @} */

/**
 * @struct ORIENTATION
 * @brief Device orientation angles
 * 
 * @var ORIENTATION::pitch
 * Pitch angle in degrees
 * @var ORIENTATION::yaw
 * Yaw angle in degrees
 * @var ORIENTATION::roll
 * Roll angle in degrees
 */

/**
 * @struct GPS_DATA
 * @brief Complete GPS data structure
 * 
 * Contains timestamp, position, altitude, speed, and signal quality information
 * from GPS module.
 * 
 * @var GPS_DATA::ts
 * Timestamp in milliseconds
 * @var GPS_DATA::date
 * Date value
 * @var GPS_DATA::time
 * Time value
 * @var GPS_DATA::lat
 * Latitude in decimal degrees
 * @var GPS_DATA::lng
 * Longitude in decimal degrees
 * @var GPS_DATA::alt
 * Altitude in meters
 * @var GPS_DATA::speed
 * Speed in knots
 * @var GPS_DATA::heading
 * Heading in degrees (0-359)
 * @var GPS_DATA::hdop
 * Horizontal dilution of precision
 * @var GPS_DATA::sat
 * Number of satellites in view
 * @var GPS_DATA::sentences
 * Total GPS sentences received
 * @var GPS_DATA::errors
 * Number of erroneous sentences
 */

/**
 * @class CLink
 * @brief Abstract base class for communication link interfaces
 * 
 * Defines the interface for various communication protocols (UART, SPI, etc.)
 * used in Freematics devices.
 */

/**
 * @class CFreematics
 * @brief Abstract base class for Freematics telematics device operations
 * 
 * Provides the core interface for xBee module communication, including
 * UART initialization, data transmission/reception, and power management.
 */
/*************************************************************************
* Base class for Freematics telematics products
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2017-2018 Stanley Huang <stanley@freematics.com.au
*************************************************************************/

#ifndef FREEMATICS_BASE
#define FREEMATICS_BASE

#include <Arduino.h>

// non-OBD/custom PIDs (no mode number)
#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11
#define PID_GPS_HDOP 0x12
#define PID_ACC 0x20
#define PID_GYRO 0x21
#define PID_COMPASS 0x22
#define PID_BATTERY_VOLTAGE 0x24
#define PID_ORIENTATION 0x25

// custom PIDs
#define PID_TIMESTAMP 0
#define PID_TRIP_DISTANCE 0x30
#define PID_DATA_SIZE 0x80
#define PID_CSQ 0x81
#define PID_DEVICE_TEMP 0x82
#define PID_DEVICE_HALL 0x83
#define PID_EXT_SENSOR1 0x90
#define PID_EXT_SENSOR2 0x91

typedef struct {
	float pitch;
	float yaw;
	float roll;
} ORIENTATION;

typedef struct {
	uint32_t ts;
	uint32_t date;
	uint32_t time;
	float lat;
	float lng;
	float alt; /* meter */
	float speed; /* knot */
	uint16_t heading; /* degree */
	uint8_t hdop;
	uint8_t sat;
	uint16_t sentences;
	uint16_t errors;
} GPS_DATA;

class CLink
{
public:
	virtual ~CLink() {}
	virtual bool begin(unsigned int baudrate = 0, int rxPin = 0, int txPin = 0) { return true; }
	virtual void end() {}
	// send command and receive response
	virtual int sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout) { return 0; }
	// receive data from SPI
	virtual int receive(char* buffer, int bufsize, unsigned int timeout) { return 0; }
	// write data to SPI
	virtual bool send(const char* str) { return false; }
	virtual int read() { return -1; }
};

class CFreematics
{
public:
	virtual void begin() {}
	// start xBee UART communication
	virtual bool xbBegin(unsigned long baudrate = 115200L, int pinRx = 0, int pinTx = 0) = 0;
	virtual void xbEnd() {}
	// read data to xBee UART
	virtual int xbRead(char* buffer, int bufsize, unsigned int timeout = 1000) = 0;
	// send data to xBee UART
	virtual void xbWrite(const char* cmd) = 0;
  // send data to xBee UART
	virtual void xbWrite(const char* data, int len) = 0;
	// receive data from xBee UART (returns 0/1/2)
	virtual int xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0) = 0;
	// purge xBee UART buffer
	virtual void xbPurge() = 0;
	// toggle xBee module power
	virtual void xbTogglePower(unsigned int duration = 500) = 0;
};

#endif
