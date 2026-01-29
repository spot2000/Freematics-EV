#ifndef CAN_DATA_H_INCLUDED
#define CAN_DATA_H_INCLUDED

#include <stdint.h>

struct AbrpTelemetry {
    // High priority parameters
    bool utc_valid = false;
    uint32_t utc = 0; // seconds since epoch

    bool soc_valid = false;
    float soc = 0.0f; // percentage

    bool power_valid = false;
    float power = 0.0f; // kW

    bool speed_valid = false;
    float speed = 0.0f; // km/h

    bool lat_valid = false;
    float lat = 0.0f; // degrees

    bool lon_valid = false;
    float lon = 0.0f; // degrees

    bool is_charging_valid = false;
    bool is_charging = false;

    bool is_dcfc_valid = false;
    bool is_dcfc = false;

    bool is_parked_valid = false;
    bool is_parked = false;

    // Lower priority parameters
    bool capacity_valid = false;
    float capacity = 0.0f; // kWh

    bool soe_valid = false;
    float soe = 0.0f; // kWh

    bool soh_valid = false;
    float soh = 0.0f; // percentage

    bool heading_valid = false;
    float heading = 0.0f; // degrees

    bool elevation_valid = false;
    float elevation = 0.0f; // meters

    bool ext_temp_valid = false;
    float ext_temp = 0.0f; // Celsius

    bool batt_temp_valid = false;
    float batt_temp = 0.0f; // Celsius

    bool voltage_valid = false;
    float voltage = 0.0f; // volts

    bool current_valid = false;
    float current = 0.0f; // amps

    bool odometer_valid = false;
    float odometer = 0.0f; // km

    bool est_battery_range_valid = false;
    float est_battery_range = 0.0f; // km

    bool hvac_power_valid = false;
    float hvac_power = 0.0f; // kW

    bool hvac_setpoint_valid = false;
    float hvac_setpoint = 0.0f; // Celsius

    bool cabin_temp_valid = false;
    float cabin_temp = 0.0f; // Celsius

    bool tire_pressure_fl_valid = false;
    float tire_pressure_fl = 0.0f; // kPa

    bool tire_pressure_fr_valid = false;
    float tire_pressure_fr = 0.0f; // kPa

    bool tire_pressure_rl_valid = false;
    float tire_pressure_rl = 0.0f; // kPa

    bool tire_pressure_rr_valid = false;
    float tire_pressure_rr = 0.0f; // kPa
};

extern AbrpTelemetry abrpTelemetry;

void resetAbrpTelemetry(AbrpTelemetry& data);

#endif // CAN_DATA_H_INCLUDED
