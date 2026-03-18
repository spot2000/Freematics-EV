#ifndef CAN_UDS_H
#define CAN_UDS_H

#include <stddef.h>
#include <stdint.h>
#include <WString.h>

bool readUDS_DID(uint32_t canId, uint32_t did, String& outResponse);
bool parseObdBufToPayload(const char* buf, char* outPayload, size_t outPayloadSize);

#endif  // CAN_UDS_H
