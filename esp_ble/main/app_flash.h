#ifndef __APP_FLASH_H
#define __APP_FLASH_H
void app_flash_write_ble_addr(int index, char* ble_addr);
void app_flash_read_ble_addr(int index, char* ble_addr);
void app_flash_erase_all(void);
#endif
