# Hitta var OBD-II Service 01 skickas

Den här koden bygger OBD-frågor i `COBD::readPID()` genom att formatera strängen:

- `sprintf(buffer, "%02X%02X\\r", dataMode, pid);`

`dataMode` är standard `1` (Mode/Service 01), så varje `readPID(pid, ...)` blir i praktiken `01xx`.

## Viktiga anropsställen

1. **Löpande polling** i `processOBD()`:
   - Hämtar PID från `obdData[]`
   - Kallar `obd.readPID(pid, value)`

2. **Kommandogränssnitt** i `telelogger.cpp`:
   - Grenen `else if (!memcmp(cmd, "01", 2))` tolkar t.ex. `0100`, `010C`, `010D`
   - Om PID inte redan finns i cache kallas `obd.readPID(pid, value)` direkt.

## Om just `0100`

`0100` skickas när `pid == 0x00` går in i `readPID()`. Det kan alltså triggas via kommandogränssnittet om ett inkommande kommando börjar med `01` och PID-delen är `00`.
