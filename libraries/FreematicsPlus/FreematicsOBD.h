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
 * @brief Strippar den första raden från bufferten och flyttar kvarvarande data.
 * @param buffer Bufferten med mottagen text att modifiera in-place.
 * @param len Antal tecken i bufferten.
 * @return Antal tecken som togs bort från början av bufferten.
 */
int dumpLine(char* buffer, int len);
/**
 * @brief Tolkar upp till fyra hextecken (med ev. mellanslag) till ett 16-bitars heltal.
 * @param p Pekare till en sträng som börjar med hextecken.
 * @return Det tolkade 16-bitarsvärdet.
 */
uint16_t hex2uint16(const char *p);
/**
 * @brief Tolkar två hextecken till ett 8-bitars heltal.
 * @param p Pekare till en sträng som börjar med hextecken.
 * @return Det tolkade 8-bitarsvärdet, eller 0 vid ogiltig input.
 */
byte hex2uint8(const char *p);

class COBD
{
public:
	/**
	 * @brief Binder en länkimplementation som används för kommunikation.
	 * @param link Pekare till CLink-objekt som hanterar den fysiska länken.
	 */
	void begin(CLink* link) { this->link = link; }
	/**
	 * @brief Initierar OBD-II-anslutningen mot ECU.
	 * @param protocol Önskat OBD-protokoll (AUTO som standard).
	 * @param quick Om true görs en snabbare init med färre försök.
	 * @return true om initiering lyckades, annars false.
	 */
	bool init(OBD_PROTOCOLS protocol = PROTO_AUTO, bool quick = false);
	/**
	 * @brief Återställer OBD-II-adaptern (mjuk reset).
	 */
	void reset();
	/**
	 * @brief Avslutar OBD-II-anslutningen/graceful shutdown.
	 */
	void uninit();
	/**
	 * @brief Sätter seriell baudrate för den underliggande länken.
	 * @param baudrate Ny baudrate i bps.
	 * @return true om baudrate sattes, annars false.
	 */
	bool setBaudRate(unsigned long baudrate);
	/**
	 * @brief Hämtar aktuell anslutningsstatus.
	 * @return OBD_STATES för anslutningen.
	 */
	OBD_STATES getState() { return m_state; }
	/**
	 * @brief Läser ett specifikt OBD-II PID-värde.
	 * @param pid PID att läsa.
	 * @param result Referens som fylls med normaliserat resultat.
	 * @return true om värdet lästes korrekt, annars false.
	 */
	bool readPID(byte pid, int& result);
	/**
	 * @brief Läser flera PID:er i följd.
	 * @param pid Array med PID:er att läsa.
	 * @param count Antal PID:er i arrayen.
	 * @param result Array som fylls med normaliserade resultat.
	 * @return Antal PID:er som lästes framgångsrikt.
	 */
	byte readPID(const byte pid[], byte count, int result[]);
	/**
	 * @brief Sätter OBD-adaptern i lågströmsläge.
	 */
	void enterLowPowerMode();
	/**
	 * @brief Väcker OBD-adaptern från lågströmsläge.
	 */
	void leaveLowPowerMode();
	/**
	 * @brief Läser diagnostiska felkoder (DTC).
	 * @param codes Buffer som fylls med felkoder.
	 * @param maxCodes Max antal felkoder som kan lagras i bufferten.
	 * @return Antal felkoder som lästes.
	 */
	int readDTC(uint16_t codes[], byte maxCodes = 1);
	/**
	 * @brief Rensar diagnostiska felkoder i ECU.
	 */
	void clearDTC();
	/**
	 * @brief Hämtar batterispänning från adaptern (fungerar utan ECU).
	 * @return Spänning i volt, eller 0 vid fel.
	 */
	float getVoltage();
	/**
	 * @brief Hämtar VIN som sträng.
	 * @param buffer Buffer som fylls med VIN.
	 * @param bufsize Storlek på bufferten (bör vara >= OBD_RECV_BUF_SIZE).
	 * @return true om VIN hämtades, annars false.
	 */
	bool getVIN(char* buffer, byte bufsize);
	/**
	 * @brief Kontrollerar om en PID stöds av ECU.
	 * @param pid PID att kontrollera.
	 * @return true om PID stöds, annars false.
	 */
	bool isValidPID(byte pid);
	/**
	 * @brief Sätter anpassat CAN header-ID (för CAN-ramar).
	 * @param num CAN header-ID.
	 */
	void setHeaderID(uint32_t num);
	/**
	 * @brief Slår på/av CAN-sniffning.
	 * @param enabled true för att aktivera sniffning, false för att stänga av.
	 */
	void sniff(bool enabled = true);
	/**
	 * @brief Sätter CAN-headerfilter för sniffning.
	 * @param num Filter-ID.
	 */
	void setHeaderFilter(uint32_t num);
	/**
	 * @brief Sätter CAN-headerfiltermask för sniffning.
	 * @param bitmask Filtermask.
	 */
	void setHeaderMask(uint32_t bitmask);
	/**
	 * @brief Tar emot sniffad CAN-data och avkodar till bytes.
	 * @param buf Buffer som fylls med rå bytes.
	 * @param len Max antal bytes att läsa.
	 * @return Antal bytes som extraherades till buf.
	 */
	int receiveData(byte* buf, int len);
	/**
	 * @brief Sätter CAN-ID för efterföljande sändningar.
	 * @param id CAN-ID att använda.
	 */
	void setCANID(uint16_t id);
	/**
	 * @brief Skickar en CAN-ram via adaptern.
	 * @param msg Buffer med CAN-data.
	 * @param len Antal bytes i msg.
	 * @param buf Buffer för svar från adaptern.
	 * @param bufsize Storlek på svarsbuffert.
	 * @return Antal svarstecken från adaptern, eller 0 vid fel.
	 */
	int sendCANMessage(byte msg[], int len, char* buf, int bufsize);
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
	 * @brief Kallas under väntetid för att ge CPU tid/handle bakgrundsjobb.
	 */
	virtual void idleTasks() { delay(5); }
	/**
	 * @brief Läser råsvar från adaptern och returnerar pekare till datafält.
	 * @param pid In/ut: önskat PID, eller 0 för att acceptera första hittade PID.
	 * @param buffer Buffer att läsa svar till.
	 * @param bufsize Storlek på buffer.
	 * @return Pekare till datafältet i buffer, eller null vid fel.
	 */
	char* getResponse(byte& pid, char* buffer, byte bufsize);
	/**
	 * @brief Tolkar ett datafält till procentvärde (0-100).
	 * @param data Pekare till hexdata.
	 * @return Normaliserat procentvärde.
	 */
	uint8_t getPercentageValue(char* data);
	/**
	 * @brief Tolkar två bytes (4 hextecken) till ett 16-bitarsvärde.
	 * @param data Pekare till hexdata.
	 * @return 16-bitarsvärde.
	 */
	uint16_t getLargeValue(char* data);
	/**
	 * @brief Tolkar en byte (2 hextecken) till ett 8-bitarsvärde.
	 * @param data Pekare till hexdata.
	 * @return 8-bitarsvärde.
	 */
	uint8_t getSmallValue(char* data);
	/**
	 * @brief Tolkar en temperaturbyte enligt OBD (A-40).
	 * @param data Pekare till hexdata.
	 * @return Temperatur i grader C.
	 */
	int16_t getTemperatureValue(char* data);
	/**
	 * @brief Normaliserar rådata enligt PID-specifikation.
	 * @param pid PID som avgör normaliseringen.
	 * @param data Pekare till hexdata.
	 * @return Normaliserat heltalsvärde.
	 */
	int normalizeData(byte pid, char* data);
	/**
	 * @brief Kontrollerar om svaret innehåller ett felmeddelande.
	 * @param buffer Svarstext från adaptern.
	 * @return 0 om inget fel, annars felkod (>0).
	 */
	byte checkErrorMessage(const char* buffer);
	/**
	 * @brief Hittar första numeriska värdet i ett svar.
	 * @param buf Svarstext att söka i.
	 * @return Pekare till värdet i buf, eller null vid fel.
	 */
	char* getResultValue(char* buf);
	OBD_STATES m_state = OBD_DISCONNECTED;
};

#endif
