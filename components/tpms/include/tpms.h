#include "stdint.h"

typedef enum {
    TIRE_FL,
    TIRE_FR,
    TIRE_RL,
    TIRE_RR
} tpms_t;

typedef struct {
    tpms_t tire;
    float pressure;
    float temperature;
    float battery;
} tpms_data_t;

typedef uint8_t tpms_ble_addr[11]; // TPMS BLE address

uint8_t tpms_connected; // TPMS connected flag

uint8_t FL_updated; // Front Left updated flag
uint8_t FR_updated; // Front Right updated flag
uint8_t RL_updated; // Rear Left updated flag
uint8_t RR_updated; // Rear Right updated flag

float FL_pressure; // Front Left pressure value
float FR_pressure; // Front Right pressure value
float RL_pressure; // Rear Left pressure value
float RR_pressure; // Rear Right pressure value

float FL_temperature; // Front Left temperature value
float FR_temperature; // Front Right temperature value
float RL_temperature; // Rear Left temperature value
float RR_temperature; // Rear Right temperature value

float FL_battery; // Front Left battery value
float FR_battery; // Front Right battery value
float RL_battery; // Rear Left battery value
float RR_battery; // Rear Right battery value

void tpms_init(void);

float tpms_get_pressure(tpms_t tire);
float tpms_get_temperature(tpms_t tire);
float tpms_get_battery(tpms_t tire);