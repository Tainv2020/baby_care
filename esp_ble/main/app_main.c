#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "common_ble.h"
#include "app_ble.h"
#include "app_uart.h"
#include "esp_log.h"
#include "app_timer.h"
#include "app_convert.h"

#define MAX_DEVICE_NUM 8
uint32_t g_arr_temparature[MAX_DEVICE_NUM];
uint8_t g_arr_battery[MAX_DEVICE_NUM];

static const char *TAG = "MAIN_APP";
static const char *TAG_GET = "MAIN_APP_GET";
static const char *TAG_POST = "MAIN_APP_POST";
static uint8_t index_for_connected_to_peer = 0;
//static uint8_t index_pre_for_connected_to_peer = 0;
extern ble_device_inst_t ble_device_table[GATTS_SUPPORT];
static bool g_get_http_status = false;
static bool g_post_http_status = false;
static bool g_post_http_start = false;

/* Status after read devices */
static bool g_status_read_all_device[MAX_DEVICE_NUM] = {false, false, false, false, false, false, false, false};

esp_bd_addr_t device1 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device2 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device3 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device4 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device5 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device6 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device7 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
esp_bd_addr_t device8 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* Declare function */
/* Event handler */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
/* Function Parse data */
static void app_parse_data_from_uart(uint8_t data[], uint32_t length);
/* Function parse data to confirm POST is ok */
static bool app_confirm_post_is_ok(uint8_t data[], uint8_t length);
/* UART0 & UART1 callback function */
void app_uart_rx_data_callback(uint8_t uart_instance, uint8_t *dta, uint16_t length);
/* Timer callback */
void vTimerCallback(TimerHandle_t pxTimer);
/* Check status read all devices */
static bool app_status_read_all_devices(void);
/* Update status all devices */
static void app_update_status_all_devices(bool status);

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
    app_uart1_sim800_init(UART1, 115200, TXD_UART1_PIN, RXD_UART1_PIN, 5);
    /* Start timer2 to get data from HTTP */
    timeout_for_get_data_from_http_start();

    while(1)
    {
        if(g_post_http_start)
        {
            g_post_http_start = false;
            /* Start POST */
            ESP_LOGE(TAG,"Start POST data to server");
            /* POST data to server */
            //app_uart_post(ble_device_table[index_pre_for_connected_to_peer].ble_addr, ble_device_table[index_pre_for_connected_to_peer].ble_data);
        }
        vTaskDelay(1);
    }
}

/* Event handler */
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

            /* Update teamparature and battery to array */
            g_arr_temparature[index_for_connected_to_peer] = ble_device_table[index_for_connected_to_peer].ble_data.temperature;
            g_arr_battery[index_for_connected_to_peer] = ble_device_table[index_for_connected_to_peer].ble_data.battery_level;
            ESP_LOGE(TAG,"NUM: %d, tem: %d, bat: %d", index_for_connected_to_peer, ble_device_table[index_for_connected_to_peer].ble_data.temperature, ble_device_table[index_for_connected_to_peer].ble_data.battery_level);
            g_status_read_all_device[index_for_connected_to_peer] = true;

            /* Verify all device was read */
            if(app_status_read_all_devices())
            {
                g_post_http_start = true;
                /* Stop timer 0 */
                timeout_for_read_data_stop();
                /* Stop timer 1 */
                time_stop_to_connect_device_next_start();
            }

            if(!g_post_http_start)
            {
                bool found = false;
                do
                {
                    index_for_connected_to_peer += 1;
                    if(index_for_connected_to_peer >= MAX_DEVICE_NUM)
                    {
                        index_for_connected_to_peer = 0;
                    }
                } while (!g_status_read_all_device[index_for_connected_to_peer]);
                
                if(ble_device_table[index_for_connected_to_peer].slot_is_used)
                {
                    found = true;
                    ESP_LOGW(TAG,"Found. Wait 3s..");
                    app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                    time_wait_to_connect_device_next_start();       //wait 3s
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
                            ESP_LOGW(TAG,"Found. Wait 3s..");
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
}

void vTimerCallback( TimerHandle_t pxTimer )
{
    int32_t lArrayIndex;

    // Optionally do something if the pxTimer parameter is NULL.
    configASSERT( pxTimer );
     // Which timer expired?
    lArrayIndex = ( int32_t ) pvTimerGetTimerID( pxTimer ); 

    if((lArrayIndex == 0) && g_get_http_status && !g_post_http_start) /* Timer 0 */
    {
        ESP_LOGE(TAG,"Timeout. Can't connect to device");
        ESP_LOGW(TAG,"Try next..");
        bool found = false;
        do
        {
            index_for_connected_to_peer += 1;
            if(index_for_connected_to_peer >= MAX_DEVICE_NUM)
            {
                index_for_connected_to_peer = 0;
            }
        } while (!g_status_read_all_device[index_for_connected_to_peer]);
        
        if(ble_device_table[index_for_connected_to_peer].slot_is_used)
        {
            found = true;
            ESP_LOGW(TAG,"Found. Wait 3s..");
            app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
            time_wait_to_connect_device_next_start();       //wait 3s
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
                    ESP_LOGW(TAG,"Found. Wait 3s..");
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
        ESP_LOGE(TAG,"Start GET data from HTTP");
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
    if(uart_instance == UART1) /* UART1 */
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
            if((length > 15) && (length <= 30))
            {
                ESP_LOGI(TAG_POST, "%s", dta);
                /* Verify after POST */
                if(app_confirm_post_is_ok(dta, length))
                {
                    /* Reset all status devices */
                    app_update_status_all_devices(false);
                    /* Start timer to scan next device */
                    time_wait_to_connect_device_next_start();
                }
                else /* If POST falure, enable g_post_http_start to POST again */
                {
                    g_post_http_start = true;
                }
            }
        }
    }
}

/* Parse data GET */
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

static bool app_confirm_post_is_ok(uint8_t data[], uint8_t length)
{
    bool retVal = true;
    uint8_t counter = 0;
    char data_compare[] = "  +HTTPACTION:  ,200,   \r\n";

    // for(counter = 0; counter < length; counter ++)
    // {
    //     ESP_LOGI(TAG_POST, "%c %c", data[counter], data_compare[counter]); 
    // }
    if((data[18] == data_compare[18]) && (data[19] == data_compare[19]) && (data[20] == data_compare[20]))
    {
        ESP_LOGI(TAG_POST, "%c%c%c", data[18], data[19], data[20]);
        ESP_LOGW(TAG_POST, "POST to HTTP success");
    }
    else
    {
        ESP_LOGE(TAG_POST, "POST to HTTP faile");
    }

    return retVal;
}

/* Check status read all devices */
static bool app_status_read_all_devices(void)
{
    bool retVal = true;
    uint8_t count = 0;
    
    for(count = 0; count < MAX_DEVICE_NUM; count ++)
    {
        if(!g_status_read_all_device[count])
        {
            retVal = false;
            break;
        }
    }

    return retVal;
}

/* Update status all devices */
static void app_update_status_all_devices(bool status)
{
    uint8_t count = 0;

    for(count = 0; count < MAX_DEVICE_NUM; count ++)
    {
        g_status_read_all_device[count] = status;
    }
}