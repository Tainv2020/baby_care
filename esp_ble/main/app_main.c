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
#include "app_convert.h"

static const char *TAG = "MAIN_APP";
static const char *TAG_GET = "MAIN_APP_GET";
static const char *TAG_POST = "MAIN_APP_POST";
static uint8_t index_for_connected_to_peer = 0;
extern ble_device_inst_t ble_device_table[GATTS_SUPPORT];
static bool g_get_http_status = false;

esp_bd_addr_t device1 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device2 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device3 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device4 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* Declare function */
/* Event handler */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
/* Function Parse data */
static void app_parse_data_from_uart(uint8_t data[], uint32_t length);
/* UART0 & UART1 callback function */
void app_uart_rx_data_callback(uint8_t uart_instance, uint8_t *dta, uint16_t length);
/* Timer callback */
void vTimerCallback(TimerHandle_t pxTimer);

/* Main app function */
void app_main(void)
{
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
    /* Call back function UART1 */
    app_uart_set_data_callback(app_uart_rx_data_callback);

    /* Init uart for module sim800 */
    app_uart1_sim800_init(UART1, 115200, TXD_UART1_PIN, RXD_UART1_PIN, 13);
    /* Start timer2 to get data from HTTP */
    timeout_for_get_data_from_http_start();
}


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
                    ESP_LOGW(TAG,"Found. Wait 10s..");
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
                        ESP_LOGW(TAG,"Found. Wait 10s..");
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
    if((lArrayIndex == 0) && g_get_http_status) /* Timer 0 */
    {
        ESP_LOGE(TAG,"Timeout. Can't connect to device");
        ESP_LOGW(TAG,"Try next..");
        bool found = false;
        while(++index_for_connected_to_peer < GATTS_SUPPORT)
        {
            if(ble_device_table[index_for_connected_to_peer].slot_is_used)
            {
                found = true;
                ESP_LOGW(TAG,"Found. Wait 10s..");
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
                    ESP_LOGW(TAG,"Found. Wait 10s..");
                    index_for_connected_to_peer = i;
                    app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                    time_wait_to_connect_device_next_start();
                    break;
                }
            }
        }  
    }
    else if((lArrayIndex == 1) && g_get_http_status) /* Timer 1 */
    {
        ESP_LOGE(TAG,"Start scan ble");
        app_ble_start_scan(10);
    }
    else if(lArrayIndex == 2) /* Timer 2 */
    {
        /* Get from server */
        ESP_LOGE(TAG,"Start get data from HTTP");
        app_uart_get();
        /* Get HTTP success */
        g_get_http_status = true;

        int index;
        /* Add device ble */
        index = app_ble_get_index_for_add_device();
        if(index != -1)
            app_ble_add_device_to_table(index, device1);

        index = app_ble_get_index_for_add_device();
        if(index != -1)    
            app_ble_add_device_to_table(index, device2);

        index = app_ble_get_index_for_add_device();
        if(index != -1)    
            app_ble_add_device_to_table(index, device3);

        index = app_ble_get_index_for_add_device();
        if(index != -1)    
            app_ble_add_device_to_table(index, device4);

        app_ble_init();
        if(ble_device_table[index_for_connected_to_peer].slot_is_used)
        {
            app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
        }

        /* Start timer1 to connect devices */
        time_wait_to_connect_device_next_start();
    }
}

/* UART0 & UART1 callback function */
void app_uart_rx_data_callback(uint8_t uart_instance, uint8_t *dta, uint16_t length)
{
    if(uart_instance == UART0)
    {

    }
    else /* UART1 */
    {
        /* From GET process */
        if(length > 500)
        {
            // Write data back to the UART
            ESP_LOGI(TAG_GET, "%s", dta);

            /* Parse data to get MAC ID */
            app_parse_data_from_uart(dta, length);
        }
        /* From POST process */
        else
        {
            ESP_LOGI(TAG_POST, "%s", dta);
        }
    }
}

/* Parse data */
static void app_parse_data_from_uart(uint8_t data[], uint32_t length)
{
    uint32_t count = 0;
    uint32_t detect_MAC = 0; /* This variable will detect character :" */

    for(count = 0; count < length; count++)
    {
        if((data[count] == ':') && (data[count + 1] == '"'))
        {
            detect_MAC += 1;

            if((detect_MAC % 2) == 0)
            {
                switch (detect_MAC)
                {
                    case 2: /* First ID MAC */
                    {
                        device1[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device1[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device1[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device1[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device1[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device1[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device1[0], device1[1], device1[2], device1[3], device1[4], device1[5]);
                        break;
                    }
                    case 4: /* Second ID MAC */
                    {
                        device2[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device2[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device2[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device2[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device2[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device2[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device2[0], device2[1], device2[2], device2[3], device2[4], device2[5]);
                        break;
                    }
                    case 6: /* Third ID MAC */
                    {
                        device3[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device3[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device3[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device3[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device3[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device3[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device3[0], device3[1], device3[2], device3[3], device3[4], device3[5]);
                        break;
                    }
                    case 8: /* Four ID MAC */
                    {
                        device4[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device4[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device4[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device4[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device4[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device4[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device4[0], device4[1], device4[2], device4[3], device4[4], device4[5]);
                        break;
                    }
                    
                    default:
                        break;
                }
            }
        }
    }
}
