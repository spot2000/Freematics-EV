#ifndef ABRP_H_INCLUDED
#define ABRP_H_INCLUDED

#include <stddef.h>

#include "CAN-data.h"

size_t buildAbrpTelemetryJson(const AbrpTelemetry& data, const char* token, char* buffer, size_t bufferSize);
bool sendAbrpTelemetry(const AbrpTelemetry& data, const char* token, char* buffer, size_t bufferSize);

#endif // ABRP_H_INCLUDED
