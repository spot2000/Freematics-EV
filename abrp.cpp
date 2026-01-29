/* ABRP (Alternative Battery Range Prediction) integration */
/* This file includes functions for sending CAN-data to ABRP server*/
/* Functions to add: send CAN data to ABRP server, handle ABRP API calls */


#include "abrp.h"

#include <stdio.h>

#include "config.h"

namespace {
constexpr const char* kAbrpCarModel = "kia:ev9:23:100:awd";
constexpr const char* kAbrpEndpointFormat = "https://api.iternio.com/1/tlm/send?api_key=%s";

size_t buildAbrpTelemetryEndpoint(char* buffer, size_t bufferSize)
{
    if (!buffer || bufferSize == 0) {
        return 0;
    }
    int written = snprintf(buffer, bufferSize, kAbrpEndpointFormat, ABRP_API_KEY);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize) {
        buffer[0] = '\0';
        return 0;
    }
    return static_cast<size_t>(written);
}

// Adds a comma separator to the JSON payload if a previous field was already written.
bool appendFieldSeparator(char* buffer, size_t bufferSize, size_t* offset, bool* firstField)
{
    if (*firstField) {
        *firstField = false;
        return true;
    }
    if (*offset + 1 >= bufferSize) {
        return false;
    }
    buffer[(*offset)++] = ',';
    buffer[*offset] = '\0';
    return true;
}

// Appends a JSON string field (e.g. "car_model":"kia:ev9:23:100:awd") to the payload buffer.
bool appendStringField(char* buffer, size_t bufferSize, size_t* offset, bool* firstField,
                       const char* key, const char* value)
{
    if (!appendFieldSeparator(buffer, bufferSize, offset, firstField)) {
        return false;
    }
    if (!value) {
        value = "";
    }
    int written = snprintf(buffer + *offset, bufferSize - *offset, "\"%s\":\"%s\"", key, value);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - *offset) {
        return false;
    }
    *offset += static_cast<size_t>(written);
    return true;
}

// Appends a JSON integer field (e.g. "is_charging":1) to the payload buffer.
bool appendIntField(char* buffer, size_t bufferSize, size_t* offset, bool* firstField,
                    const char* key, int value)
{
    if (!appendFieldSeparator(buffer, bufferSize, offset, firstField)) {
        return false;
    }
    int written = snprintf(buffer + *offset, bufferSize - *offset, "\"%s\":%d", key, value);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - *offset) {
        return false;
    }
    *offset += static_cast<size_t>(written);
    return true;
}

// Appends a JSON unsigned integer field (e.g. "utc":1690000000) to the payload buffer.
bool appendUIntField(char* buffer, size_t bufferSize, size_t* offset, bool* firstField,
                     const char* key, unsigned int value)
{
    if (!appendFieldSeparator(buffer, bufferSize, offset, firstField)) {
        return false;
    }
    int written = snprintf(buffer + *offset, bufferSize - *offset, "\"%s\":%u", key, value);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - *offset) {
        return false;
    }
    *offset += static_cast<size_t>(written);
    return true;
}

// Appends a JSON floating-point field with a provided printf format (e.g. "soc":87.50).
bool appendFloatField(char* buffer, size_t bufferSize, size_t* offset, bool* firstField,
                      const char* key, float value, const char* format)
{
    if (!appendFieldSeparator(buffer, bufferSize, offset, firstField)) {
        return false;
    }
    int written = snprintf(buffer + *offset, bufferSize - *offset, "\"%s\":", key);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - *offset) {
        return false;
    }
    *offset += static_cast<size_t>(written);
    written = snprintf(buffer + *offset, bufferSize - *offset, format, value);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - *offset) {
        return false;
    }
    *offset += static_cast<size_t>(written);
    return true;
}
} // namespace

char abrpUserKey[64] = ABRP_USER_KEY;

// Builds the ABRP telemetry JSON body into the supplied buffer and returns the payload size.
size_t buildAbrpTelemetryJson(const AbrpTelemetry& data, const char* token, char* buffer, size_t bufferSize)
{
    if (!buffer || bufferSize == 0) {
        return 0;
    }
    if (!token || token[0] == '\0') {
        token = abrpUserKey;
    }

    size_t offset = 0;
    int written = snprintf(buffer, bufferSize, "{\"token\":\"%s\",\"tlm\":{", token);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize) {
        buffer[0] = '\0';
        return 0;
    }
    offset = static_cast<size_t>(written);

    bool firstField = true;

    if (!appendStringField(buffer, bufferSize, &offset, &firstField, "car_model", kAbrpCarModel)) {
        return 0;
    }

    if (data.utc_valid) {
        if (!appendUIntField(buffer, bufferSize, &offset, &firstField, "utc", data.utc)) {
            return 0;
        }
    }
    if (data.soc_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "soc", data.soc, "%.2f")) {
            return 0;
        }
    }
    if (data.power_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "power", data.power, "%.2f")) {
            return 0;
        }
    }
    if (data.speed_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "speed", data.speed, "%.2f")) {
            return 0;
        }
    }
    if (data.lat_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "lat", data.lat, "%.6f")) {
            return 0;
        }
    }
    if (data.lon_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "lon", data.lon, "%.6f")) {
            return 0;
        }
    }
    if (data.is_charging_valid) {
        if (!appendIntField(buffer, bufferSize, &offset, &firstField, "is_charging", data.is_charging ? 1 : 0)) {
            return 0;
        }
    }
    if (data.is_dcfc_valid) {
        if (!appendIntField(buffer, bufferSize, &offset, &firstField, "is_dcfc", data.is_dcfc ? 1 : 0)) {
            return 0;
        }
    }
    if (data.is_parked_valid) {
        if (!appendIntField(buffer, bufferSize, &offset, &firstField, "is_parked", data.is_parked ? 1 : 0)) {
            return 0;
        }
    }

    if (data.capacity_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "capacity", data.capacity, "%.2f")) {
            return 0;
        }
    }
    if (data.soe_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "soe", data.soe, "%.2f")) {
            return 0;
        }
    }
    if (data.soh_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "soh", data.soh, "%.2f")) {
            return 0;
        }
    }
    if (data.heading_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "heading", data.heading, "%.2f")) {
            return 0;
        }
    }
    if (data.elevation_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "elevation", data.elevation, "%.2f")) {
            return 0;
        }
    }
    if (data.ext_temp_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "ext_temp", data.ext_temp, "%.2f")) {
            return 0;
        }
    }
    if (data.batt_temp_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "batt_temp", data.batt_temp, "%.2f")) {
            return 0;
        }
    }
    if (data.voltage_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "voltage", data.voltage, "%.2f")) {
            return 0;
        }
    }
    if (data.current_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "current", data.current, "%.2f")) {
            return 0;
        }
    }
    if (data.odometer_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "odometer", data.odometer, "%.2f")) {
            return 0;
        }
    }
    if (data.est_battery_range_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "est_battery_range",
                               data.est_battery_range, "%.2f")) {
            return 0;
        }
    }
    if (data.hvac_power_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "hvac_power", data.hvac_power, "%.2f")) {
            return 0;
        }
    }
    if (data.hvac_setpoint_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "hvac_setpoint", data.hvac_setpoint, "%.2f")) {
            return 0;
        }
    }
    if (data.cabin_temp_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "cabin_temp", data.cabin_temp, "%.2f")) {
            return 0;
        }
    }
    if (data.tire_pressure_fl_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "tire_pressure_fl",
                               data.tire_pressure_fl, "%.2f")) {
            return 0;
        }
    }
    if (data.tire_pressure_fr_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "tire_pressure_fr",
                               data.tire_pressure_fr, "%.2f")) {
            return 0;
        }
    }
    if (data.tire_pressure_rl_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "tire_pressure_rl",
                               data.tire_pressure_rl, "%.2f")) {
            return 0;
        }
    }
    if (data.tire_pressure_rr_valid) {
        if (!appendFloatField(buffer, bufferSize, &offset, &firstField, "tire_pressure_rr",
                               data.tire_pressure_rr, "%.2f")) {
            return 0;
        }
    }

    if (offset + 2 >= bufferSize) {
        return 0;
    }
    buffer[offset++] = '}';
    buffer[offset++] = '}';
    buffer[offset] = '\0';
    return offset;
}

// Constructs the ABRP payload and (once implemented) sends it to the ABRP telemetry endpoint.
bool sendAbrpTelemetry(const AbrpTelemetry& data, const char* token, char* buffer, size_t bufferSize)
{
    size_t payloadSize = buildAbrpTelemetryJson(data, token, buffer, bufferSize);
    if (payloadSize == 0) {
        return false;
    }

    char endpoint[128];
    if (buildAbrpTelemetryEndpoint(endpoint, sizeof(endpoint)) == 0) {
        return false;
    }

    // TODO: Add HTTP(S)/UDP client transport wiring to send payload to ABRP endpoint.
    // Example target: https://api.iternio.com/1/tlm/send?api_key=... with JSON payload built above.
    // TODO: Inject network client dependency (TeleClientHTTP/UDP) and handle retries/backoff.

    return false;
}
