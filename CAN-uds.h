#ifndef CAN_UDS_H
#define CAN_UDS_H

#include <stddef.h>
#include <stdint.h>
#include <WString.h>

bool read_UDS(uint32_t txCanId,
              const char* udsRequestHex,
              char* outRespTxt, size_t outRespTxtSize,
              uint8_t* outRespBytes, size_t outRespBytesMax, size_t* outRespLen);

String UDS_read_DID(const char* canIdHex, const char* didHex);

#endif  // CAN_UDS_H
