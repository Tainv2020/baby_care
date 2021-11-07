#include "app_flash.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#define DEVICE_SUPPORT 10
#define __STORAGE_NVS_USER   "__nvs_user"

nvs_handle nvs_handle_user;
static char DeviceKey[DEVICE_SUPPORT][30] = {
    "ble_addr_0",
    "ble_addr_1",
    "ble_addr_2",
    "ble_addr_3",
    "ble_addr_4",
    "ble_addr_5",
    "ble_addr_6",
    "ble_addr_7",
    "ble_addr_8",
    "ble_addr_9",
};

void app_flash_write_ble_addr(int index, char* ble_addr)
{
    nvs_open(__STORAGE_NVS_USER, NVS_READWRITE, &nvs_handle_user);
    nvs_set_str(nvs_handle_user,DeviceKey[index], ble_addr);
}

void app_flash_read_ble_addr(int index, char* ble_addr)
{
    size_t length = 13;
    nvs_open(__STORAGE_NVS_USER, NVS_READONLY, &nvs_handle_user);
    nvs_get_str(nvs_handle_user,DeviceKey[index], ble_addr, &length);
    printf("get ble addr: %s", ble_addr); 
}

void app_flash_erase_all(void)
{
    ESP_ERROR_CHECK(nvs_flash_erase());
}
