# Kodgenomgång av Freematics‑EV

Det här dokumentet beskriver koden i projektet **Freematics‑EV** i detalj, inklusive inbyggda bibliotek. Fokus är på hur telemetridata samlas in, buffras, lagras och skickas samt vilka bygg-/konfigurationsfiler som styr beteendet.

## Översikt av projektet

Projektet är en firmware‑sketch för Freematics ONE+ (ESP32‑baserad telematics‑enhet). Den loggar och skickar fordonsdata (OBD‑II), GNSS‑position, MEMS‑rörelsedata, signalstyrka och enhetsstatus. Datat kan lagras lokalt (SD/SPIFFS), strömmas till Freematics Hub via UDP/HTTPS och exponeras via en inbyggd HTTP‑server samt en enkel dashboard‑sida.

## Bygg och konfiguration

- **platformio.ini** beskriver PlatformIO‑miljön (`env:esp32dev`), Arduino‑ramverk, baudrate, flash‑inställningar och extra bibliotekskatalog (`lib_extra_dirs=./libraries`).
- **config.h** innehåller compile‑time‑konfiguration (t.ex. om OBD, MEMS, GNSS, Wi‑Fi, BLE och HTTPD ska vara aktiva), buffertstorlek, nätverks‑ och lagringsval samt trösklar/tidsintervall för datainsamling och standby.
- **config.xml** definierar samma inställningar som strukturerade “defines” (t.ex. via ett konfigurationsverktyg), med default‑värden för OBD, GNSS‑läge, lagring, Wi‑Fi och serverprotokoll.

### Viktiga konfigurationsparametrar (exempel)

- **OBD‑ och GNSS‑lägen** styrs av `ENABLE_OBD`, `GNSS`, `ENABLE_MEMS` i `config.h`.
- **Serverinställningar** (host, port, protokoll) styrs av `SERVER_HOST`, `SERVER_PORT`, `SERVER_PROTOCOL`.
- **Lagring** styrs av `STORAGE` (SD, SPIFFS eller ingen lagring).
- **Dataintervall** styrs av `DATA_INTERVAL_TABLE` och `STATIONARY_TIME_TABLE` för att ändra sampling när bilen står still.

## Huvudapplikation: telelogger.cpp

`telelogger.cpp` är den centrala Arduino‑sketchen som koordinerar all insamling, lagring och överföring av data.

### Struktur och globala tillstånd

- **Tillståndsflaggor** (STATE_*): håller reda på om OBD, GNSS, MEMS, nätverk och lagring är redo, samt om enheten kör aktivt eller är i standby.
- **PID‑lista** (`obdData`): definierar vilka OBD‑PID:ar som läses in och på vilken “tier” (prioritet) de pollas.
- **Bufrar**: `CBufferManager bufman` hanterar en ringbuffert av datapaket (via `CBuffer`).
- **Nätverksklient**: `TeleClientUDP` eller `TeleClientHTTP` beroende på `SERVER_PROTOCOL`.
- **Lagring**: `SDLogger` eller `SPIFFSLogger` beroende på `STORAGE`.

### Initialisering (`setup` / `initialize`)

1. **NVS och konfigurationsladdning**: NVS används för att läsa sparad konfiguration.
2. **Seriell loggning**: USB‑serial initieras.
3. **Enhets‑ID**: genereras för att identifiera enheten i loggar/telemetri.
4. **MEMS‑initialisering**: testar först `ICM_42627`, sedan `ICM_20948` och kalibrerar accelerometerbias.
5. **GNSS‑initiering**: startar GNSS, internt eller externt beroende på `GNSS` och hårdvarustöd.
6. **OBD‑initiering**: etablerar OBD‑anslutning, läser VIN och eventuella DTC:er.
7. **Lagring**: initierar SD/SPIFFS och öppnar en ny loggfil.
8. **HTTP‑server**: om aktiverad startas en lokal webserver och IP skrivs ut.
9. **BLE**: BLE‑SPP‑server initieras om aktiverad.
10. **Telemetri‑tråd**: en separat task (`telemetry`) startas för nätverkshantering och uppladdning.

### Datainsamling (`process`)

Varje cykel i `loop()` anropar `process()` som:

- **OBD‑pollning**: läser PID:ar (t.ex. hastighet, varvtal, last, temperatur) enligt prioritet, med felhantering och återförsök.
- **GNSS**: samlar lat/long, hastighet, tid, satellitstatus. Inkluderar reset om GNSS blir “stale”.
- **MEMS**: läser accelerometer/gyro/magnetometer och beräknar rörelse.
- **Enhetstemperatur och spänning**: läser batterispänning och chip‑temperatur.
- **Buffring**: skriver alla värden till `CBuffer` som sedan markeras som fylld.
- **Adaptiv sampling**: justerar `dataInterval` baserat på rörelse (stationär => glesare sampling).

### Nätverk och telemetri (`telemetry`)

`telemetry()` kör parallellt och sköter anslutningar och överföring:

- **Wi‑Fi först**: om SSID är konfigurerat försöker den ansluta via Wi‑Fi och skickar data där.
- **Cellulär fallback**: om Wi‑Fi inte finns tillgängligt, aktiveras cellulär modul (SIMCOM) för uppkoppling.
- **Ping/keep‑alive**: periodisk ping under standby, samt övervakning av RSSI.
- **Sändning**: buffrar serialiseras till `CStorageRAM`, paketeras och skickas till server via UDP/HTTP.
- **Omkoppling**: när Wi‑Fi blir tillgängligt stängs cellulärmodulen av för att spara ström.

### Standby‑logik

- Om rörelse saknas under längre tid går enheten in i **standby**.
- Standby innebär att nätverksmoduler stängs av och buffrar rensas; enheten “pingar” då och då för att se om den ska vakna.

### HTTP‑API och live‑data

Om `ENABLE_HTTPD` är aktivt exponeras följande (via `dataserver.cpp`):

- `/api/info` — CPU‑temp, RTC‑tid, storage‑info.
- `/api/live` — live OBD/GPS/MEMS‑data i JSON.
- `/api/log/<id>` — rå CSV‑loggfil.
- `/api/data/<id>?pid=...` — JSON‑export för en specifik PID.

`handlerLiveData()` i `telelogger.cpp` formaterar JSON för realtidsdata.

## Lagring: telestore.*

`CStorage` och underklasser hanterar serialisering och loggning:

- **CStorage**: skriver PID‑par till serial output (standard) eller vidare till cache/logg.
- **CStorageRAM**: buffrar data i RAM, lägger till checksum‑tail för transmissionspaket.
- **FileLogger**: gemensam filskrivning för SD/SPIFFS.
- **SDLogger/SPIFFSLogger**: initierar media, öppnar filer och flushar data.

## Buffring och telemetri‑paket: teleclient.*

- **CBuffer**: lagrar PID‑värden med typ och count i binärt format innan serialisering.
- **CBufferManager**: pool av `CBuffer`‑slots i RAM/PSRAM med logik för “äldst vinner” när bufferten är full.
- **TeleClient**: abstrakt klient med räknare för tx/rx.
- **TeleClientUDP/HTTP**: konkret implementation som skickar datapaket via Wi‑Fi eller cellulär.

## HTTP‑serverbibliotek: libraries/httpd

Det här är en minimal HTTP‑serverimplementation (MiniWeb):

- **httpd.h**: typer, flaggor, request/response‑strukturer och HTTP‑status.
- **httpd.c/httppil.c/httpjson.c**: implementerar socket‑hantering, request‑parsing och JSON‑helpers.
- Servern används av `dataserver.cpp` för API:er.

## FreematicsPlus‑biblioteket

Det interna biblioteket i `libraries/FreematicsPlus` är kärnan för hårdvarunära funktionalitet.

### FreematicsBase.h

- Definierar **egna PIDs** för GPS, MEMS, spänning, temperatur, etc.
- Definierar `GPS_DATA`, `ORIENTATION` och abstrakta länkar (`CLink`) och enhetsbas (`CFreematics`).

### FreematicsPlus.h / FreematicsPlus.cpp

- Innehåller **hårdvaru‑specifika pin‑mappar** för ESP32 och Freeematics‑enheterna.
- `FreematicsESP32` implementerar GPS‑, UART‑, buzzer‑, reset‑ och xBee‑kommunikation samt styrning av co‑processor.

### FreematicsNetwork.h / FreematicsNetwork.cpp

- Implementerar **Wi‑Fi‑** och **cellulära** klienter:
  - `ClientWIFI`, `WifiUDP`, `WifiHTTP`.
  - `CellSIMCOM`, `CellUDP`, `CellHTTP` för SIM7600/7070/5360.
- Hanterar APN, signalstyrka, IP‑uppslagning och GNSS‑data via modem.

### FreematicsOBD.h / FreematicsOBD.cpp

- `COBD` hanterar OBD‑initialisering, PID‑läsning, DTC‑hantering, CAN‑sniffing och VIN‑läsning.
- Parserar svar från ECU och normaliserar data.

### FreematicsMEMS.h / FreematicsMEMS.cpp

- Stöd för MEMS‑sensorer (ICM‑42627, ICM‑20948 m.fl.).
- Levererar accelerometer/gyro/magnetometer‑data och orientering.

### FreematicsGPS.h / FreematicsGPS.cpp

- Baserat på TinyGPS: parsar NMEA‑strömmar, beräknar position, hastighet, kurs, satelliter, HDOP, etc.

### Utility‑filer

`libraries/FreematicsPlus/utility` innehåller registerdefinitioner och C‑drivrutiner för MEMS‑sensorer samt BLE SPP‑server.

## Dashboard (webb)

I mappen `dashboard/` finns en enkel HTML/JS‑dashboard:

- **index.html**: visar enhets‑ och statusvärden.
- **dashboard.js**: parsar seriell/kommunikations‑output och uppdaterar UI‑fält i realtid, inklusive ikonstatus för GNSS/OBD/Wi‑Fi.

## Sammanfattning av dataflödet

1. **Sensorer** (OBD, GNSS, MEMS) läses in.
2. **Buffring** sker i `CBuffer` via `CBufferManager`.
3. **Lagring**: datapaket serialiseras till SD/SPIFFS (valfritt).
4. **Telemetri**: datapaket paketeras i RAM‑buffer, checksum läggs till och skickas via UDP/HTTP.
5. **API/Dashboard**: HTTP‑server och enkel webbsida ger realtidsstatus.

Detta ger en komplett telemetri‑pipeline från bilens ECU och sensorer hela vägen till server, lokalt arkiv och realtids‑dashboard.
