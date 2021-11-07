#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "common_ble.h"
#include "app_ble.h"
#include "app_uart.h"
#include "esp_log.h"
#include "app_timer.h"

static const char *TAG = "MAIN_APP";
static uint8_t index_for_connected_to_peer = 0;
extern ble_device_inst_t ble_device_table[GATTS_SUPPORT];
static bool g_init_sim800_done = false;

esp_bd_addr_t device1 = {0xC9,0xAD,0x7F,0x93,0x4C,0xDE};
esp_bd_addr_t device2 = {0xFB,0x0B,0x2B,0x97,0xEA,0x0E};
esp_bd_addr_t device3 = {0x01,0x02,0x03,0x04,0x05,0x06};

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if(event_base == EVENT_BLE)
    {
        if(event_id == EVENT_BLE_START_SCAN)
        {
            ESP_LOGW(TAG,"EVENT_BLE_START_SCAN");
        }
        else if(event_id == EVENT_BLE_STOP_SCAN)
        {
            ESP_LOGW(TAG,"EVENT_BLE_STOP_SCAN");
        }
        else if(event_id == EVENT_BLE_CONNECTED)
        {
            ESP_LOGW(TAG,"EVENT_BLE_CONNECTED");
        }
        else if(event_id == EVENT_BLE_DISCONNECTED)
        {
            ESP_LOGW(TAG,"EVENT_BLE_DISCONNECTED");
        }
        else if(event_id == EVENT_BLE_GOT_DATA_DONE)
        {
            ESP_LOGW(TAG,"EVENT_BLE_GOT_DATA_DONE");
            ESP_LOGW(TAG,"Try next..");
            bool found = false;
            while(++index_for_connected_to_peer < GATTS_SUPPORT)
            {
                if(ble_device_table[index_for_connected_to_peer].slot_is_used)
                {
                    found = true;
                    ESP_LOGW(TAG,"Found. Wait 5s..");
                    app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                    time_wait_to_connect_device_next_start();       //wait 5s
                    break;
                }
            }

            if(index_for_connected_to_peer >= GATTS_SUPPORT)
            {
                index_for_connected_to_peer = 0;
            }

            if(!found)      // reset turn
            {
                for(int i=0;i<GATTS_SUPPORT;i++)
                {
                    if(ble_device_table[i].slot_is_used)
                    {
                        ESP_LOGW(TAG,"Found. Wait 5s..");
                        index_for_connected_to_peer = i;
                        app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                        time_wait_to_connect_device_next_start();
                        break;
                    }
                }
            }
        }
    }
}

void vTimerCallback( TimerHandle_t pxTimer )
{
    int32_t lArrayIndex;

    // Optionally do something if the pxTimer parameter is NULL.
    configASSERT( pxTimer );

     // Which timer expired?
    lArrayIndex = ( int32_t ) pvTimerGetTimerID( pxTimer ); 
    if((lArrayIndex == 0) && g_init_sim800_done)
    {
        ESP_LOGE(TAG,"Timeout. Can't connect to device");
        ESP_LOGW(TAG,"Try next..");
        bool found = false;
        while(++index_for_connected_to_peer < GATTS_SUPPORT)
        {
            if(ble_device_table[index_for_connected_to_peer].slot_is_used)
            {
                found = true;
                ESP_LOGW(TAG,"Found. Wait 5s..");
                app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                time_wait_to_connect_device_next_start();
                break;
            }
        }

        if(index_for_connected_to_peer >= GATTS_SUPPORT)
        {
            index_for_connected_to_peer = 0;
        }

        if(!found)      // reset turn
        {
            for(int i=0;i<GATTS_SUPPORT;i++)
            {
                if(ble_device_table[i].slot_is_used)
                {
                    ESP_LOGW(TAG,"Found. Wait 5s..");
                    index_for_connected_to_peer = i;
                    app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                    time_wait_to_connect_device_next_start();
                    break;
                }
            }
        }  
    }  
    else if((lArrayIndex == 1) && g_init_sim800_done)
    {
        ESP_LOGE(TAG,"Start scan ble");
        app_ble_start_scan(10);
    }
    else if(lArrayIndex == 2)
    {
        ESP_LOGE(TAG,"Init sim800");
        /* Init sim800 */
        app_uart_sim800_init();
        /* start timer 0 to connect device ble */
        app_ble_start_scan(10);
    }
}


void app_uart_rx_data_callback(uint8_t *dta, uint16_t length)
{
    ESP_LOG_BUFFER_HEX(TAG, dta, length);
    //uart_write_bytes for send
}

void app_main(void)
{
    int index;

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register((EVENT_BLE), ESP_EVENT_ANY_ID, &event_handler, NULL));
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    /* Init timer */
    app_timer_init(&vTimerCallback);
    
    /* Init UART0 */
    app_uart_init();
    /* Init uart for module sim800 */
    app_uart1_sim800_init(UART1, 115200, TXD_UART1_PIN, RXD_UART1_PIN, 13);
    /* Start timer init sim800 */
    timeout_for_start_init_sim800();
    /* Call back function UART1 */
    app_uart_set_data_callback(app_uart_rx_data_callback);

    index = app_ble_get_index_for_add_device();
    if(index != -1)
        app_ble_add_device_to_table(index, device1);

    index = app_ble_get_index_for_add_device();
    if(index != -1)    
        app_ble_add_device_to_table(index, device2);

    index = app_ble_get_index_for_add_device();
    if(index != -1)    
        app_ble_add_device_to_table(index, device3);

    app_ble_init();
    if(ble_device_table[index_for_connected_to_peer].slot_is_used)
    {
        app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
    }
}
