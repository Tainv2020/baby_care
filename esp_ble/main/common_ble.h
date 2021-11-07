#ifndef __COMMON_BLE_H
#define __COMMON_BLE_H
#include <stdint.h>
#include <string.h>
#include "esp_gatt_defs.h"
#include <esp_event.h>
ESP_EVENT_DECLARE_BASE(EVENT_BLE);
#define GATTS_SUPPORT    10
enum 
{
    EVENT_BLE_START_SCAN = 0,
    EVENT_BLE_SCAN_DONE = 1,
    EVENT_BLE_STOP_SCAN = 2,
    EVENT_BLE_CONNECTED = 3,
    EVENT_BLE_DISCONNECTED = 4,
    EVENT_BLE_GOT_DATA_DONE = 5
};

typedef struct  
{
    uint32_t temperature;
    uint8_t battery_level;   
}   ble_app_data_t;

typedef struct
{
    int ble_index;
    esp_bd_addr_t  ble_addr;
    ble_app_data_t ble_data;
    bool slot_is_used;
}   ble_device_inst_t;

void app_ble_init(void);
void app_ble_start_scan(uint32_t duration);
void app_ble_stop_scan(void);
void app_ble_set_device_to_connect(uint8_t * ble_addr);
bool app_ble_add_device_to_table(int ble_index, uint8_t * ble_addr);
int app_ble_get_index_in_table(uint8_t * ble_addr);
int app_ble_get_index_for_add_device(void);
#endif