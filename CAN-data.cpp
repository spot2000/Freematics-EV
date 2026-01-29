/* This file includes the data structure for holding CAN data that will be used to send to ABRP and also saved to SD-card */
/* Use data structure from evDash project */


#include "abrp.h"
#include "CAN-data.h"

#include <FreematicsPlus.h>

AbrpTelemetry abrpTelemetry;

void resetAbrpTelemetry(AbrpTelemetry& data)
{
    data = AbrpTelemetry();
    // TODO: When CAN decode is implemented, populate abrpTelemetry fields from CAN PIDs here.
}
