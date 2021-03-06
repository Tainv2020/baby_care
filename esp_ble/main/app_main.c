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

#define MAX_POST_FALURE 2

static const char *TAG = "MAIN_APP";
static const char *TAG_GET = "MAIN_APP_GET";
static const char *TAG_POST = "MAIN_APP_POST";
static uint8_t g_scan_device_counter = 0;

/* Buffer temparature and battery */
uint32_t g_arr_temparature[MAX_DEVICE_NUM];
uint8_t g_arr_battery[MAX_DEVICE_NUM];
/* Status after read devices */
bool g_status_read_all_device[MAX_DEVICE_NUM] = {false, false, false, false, false, false, false, false};

/* Device array and index */
extern ble_device_inst_t ble_device_table[MAX_DEVICE_NUM];
extern uint8_t index_for_connected_to_peer;

/* Status GET and POST */
static bool g_get_http_status = false; /* To inform GET progress */
static bool g_post_http_status = false; /* To inform POST in progress */
static bool g_post_http_start = false; /* To start POST progress */
static bool g_post_http_success = false; /* POST success or falure */
static bool g_server_repsond_status = false; /* Respond from server */
static uint8_t g_post_http_falure_counter = 0; /* POST falure counter */

/* Status reading devices */
static bool g_reading_devices_status = false;

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
static void app_uart_rx_data_callback(uint8_t uart_instance, uint8_t *dta, uint16_t length);
/* Timer callback */
static void vTimerCallback(TimerHandle_t pxTimer);
/* Check status read all devices */
static uint8_t app_status_read_all_devices(void);
/* Update status all devices */
static void app_update_status_all_devices(bool status);
/* Compare MAC id */
static uint8_t app_compare_mac_id(ble_device_inst_t device_info);

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
            // ESP_LOGE(TAG, "%d %d %d %d", g_arr_temparature[0], g_arr_temparature[1], g_arr_temparature[2], g_arr_temparature[3]);
            ESP_LOGE(TAG,"Status devices: %d %d %d %d", g_status_read_all_device[0], g_status_read_all_device[1], g_status_read_all_device[2], g_status_read_all_device[3]);
            /* Start timer timeout */
            ESP_LOGE(TAG,"Start timeout for POST precess");
            timeout_for_post_data_to_http_start();
            /* POST data to server */
            app_uart_post(device1, device2, device3, device4, device5, device6, device7, device8, g_arr_temparature, g_arr_battery, g_post_http_success);
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
            ESP_LOGE(TAG,"Counter scan: %d", g_scan_device_counter);
            ESP_LOGE(TAG,"Index: %d", index_for_connected_to_peer);
            /* Times scan ble devices */
            g_scan_device_counter += 1;
            if(g_scan_device_counter > (MAX_DEVICE_NUM + 1))
            {
                g_scan_device_counter = 0;
            }
        }
        else if(event_id == EVENT_BLE_STOP_SCAN)
        {
            ESP_LOGW(TAG,"EVENT_BLE_STOP_SCAN");
            /* Is reading */
            g_reading_devices_status = true;
        }
        else if(event_id == EVENT_BLE_CONNECTED)
        {
            ESP_LOGW(TAG,"EVENT_BLE_CONNECTED");
        }
        else if(event_id == EVENT_BLE_DISCONNECTED)
        {
            ESP_LOGW(TAG,"EVENT_BLE_DISCONNECTED");
            /* Start timer scan */
            time_wait_to_connect_device_next_start();       //wait 3s
            /* Reading done */
            g_reading_devices_status = false;
        }
        else if(event_id == EVENT_BLE_GOT_DATA_DONE)
        {
            ESP_LOGW(TAG,"EVENT_BLE_GOT_DATA_DONE");
            ESP_LOGW(TAG,"Try next..");

            uint8_t index = app_compare_mac_id(ble_device_table[index_for_connected_to_peer]);

            /* Update teamparature and battery to array */
            g_arr_temparature[index] = ble_device_table[index].ble_data.temperature;
            g_arr_battery[index] = ble_device_table[index].ble_data.battery_level;
            ESP_LOGE(TAG,"NUM: %d, tem: %d, bat: %d", index, ble_device_table[index].ble_data.temperature, ble_device_table[index].ble_data.battery_level);
            ESP_LOGE(TAG, "%d:%d:%d:%d:%d:%d", ble_device_table[index].ble_addr[0], ble_device_table[index].ble_addr[1], ble_device_table[index].ble_addr[2], ble_device_table[index].ble_addr[3], ble_device_table[index].ble_addr[4], ble_device_table[index].ble_addr[5]);
            g_status_read_all_device[index] = true;

            /* Verify all device was read */
            if(app_status_read_all_devices() == (MAX_DEVICE_NUM - 1))
            {
                /* Reset index devices */
                index_for_connected_to_peer = 0;
                /* To start POST progress */
                g_post_http_start = true; 
                /* To inform POST in progress */
                g_post_http_status = true;
                g_scan_device_counter = 0;
                /* Stop timer 0 */
                timeout_for_read_data_stop();
                /* Stop timer 1 */
                time_stop_to_connect_device_next_start();
            }

            if(!g_post_http_status)
            {
                do
                {
                    index_for_connected_to_peer += 1;
                    if(index_for_connected_to_peer >= MAX_DEVICE_NUM)
                    {
                        index_for_connected_to_peer = 0;
                    }
                } while (!ble_device_table[index_for_connected_to_peer].slot_is_used);
                
                ESP_LOGW(TAG,"Found. Wait 3s..");
                app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
            }
        }
    }
}

static void vTimerCallback( TimerHandle_t pxTimer )
{
    int32_t lArrayIndex;

    // Optionally do something if the pxTimer parameter is NULL.
    configASSERT( pxTimer );
     // Which timer expired?
    lArrayIndex = ( int32_t ) pvTimerGetTimerID( pxTimer ); 

    /* Timer 0 */
    /* Run timer 0 when GET was done and POST not yet in progress and reading done */
    if((lArrayIndex == 0) && g_get_http_status && !g_post_http_status && !g_reading_devices_status)
    {
        ESP_LOGE(TAG,"Timeout. Can't connect to device");
        ESP_LOGW(TAG,"Try next..");

        if((g_scan_device_counter > MAX_DEVICE_NUM) && (app_status_read_all_devices() != 0))
        {
            /* Reset index devices */
            index_for_connected_to_peer = 0;
            /* To start POST progress */
            g_post_http_start = true; 
            /* To inform POST in progress */
            g_post_http_status = true;
            g_scan_device_counter = 0;
            /* Stop timer 0 */
            timeout_for_read_data_stop();
            /* Stop timer 1 */
            time_stop_to_connect_device_next_start();
        }

        if(!g_post_http_status)
        {
            do
            {
                index_for_connected_to_peer += 1;
                if(index_for_connected_to_peer >= MAX_DEVICE_NUM)
                {
                    index_for_connected_to_peer = 0;
                }
            } while (!ble_device_table[index_for_connected_to_peer].slot_is_used);

            ESP_LOGW(TAG,"Found. Wait 3s..");
            app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
            time_wait_to_connect_device_next_start();       //wait 3s
        }
    }
    /* Timer 1 */
    /* Run timer 1 when GET was done and POST not yet in progress  */
    else if((lArrayIndex == 1) && g_get_http_status && !g_post_http_status)
    {
        ESP_LOGE(TAG,"Start scan ble");
        app_ble_start_scan(10);
    }
    /* Timer 2 */
    /* Run once after power on */
    else if(lArrayIndex == 2)
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
    /* Timer 3 */
    /* Run timer 3 when GET was done */
    else if((lArrayIndex == 1) && g_get_http_status)
    {
        /* If nothing respond from server */
        if(!g_server_repsond_status)
        {
            ESP_LOGE(TAG,"No respond from server. Start scan devices again");
            /* Reset to run timer 0 and timer 1 */
            g_post_http_status = false;
            /* Reset all status devices */
            app_update_status_all_devices(false);
            /* Reset g_post_http_falure_counter */
            g_post_http_falure_counter = 0;
            /* Scan devices from index 0 */
            app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
            time_wait_to_connect_device_next_start();
            /* Start timer to scan next device */
            time_wait_to_connect_device_next_start(); 
        }
    }
}

/* UART0 & UART1 callback function */
static void app_uart_rx_data_callback(uint8_t uart_instance, uint8_t *dta, uint16_t length)
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
                /* Update server respond status */
                g_server_repsond_status = true;
                /* Verify after POST */
                /* If POST is ok or POST falure = limit times */
                if(app_confirm_post_is_ok(dta, length))
                {
                    /* Reset to run timer 0 and timer 1 */
                    g_post_http_status = false;
                    /* Reset all status devices */
                    app_update_status_all_devices(false);
                    /* Reset g_post_http_falure_counter */
                    g_post_http_falure_counter = 0;
                    
                    /* Scan devices from index 0 */
                    app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                    time_wait_to_connect_device_next_start();
                    /* Start timer to scan next device */
                    time_wait_to_connect_device_next_start();
                }
                else /* If POST falure, enable g_post_http_start to POST again */
                {
                    /* Update falure POST times */
                    g_post_http_falure_counter += 1;

                    if(g_post_http_falure_counter >= MAX_POST_FALURE)
                    {
                        /* Reset to run timer 0 and timer 1 */
                        g_post_http_status = false;
                        /* Reset all status devices */
                        app_update_status_all_devices(false);
                        /* Reset g_post_http_falure_counter */
                        g_post_http_falure_counter = 0;
                    
                        /* Scan devices from index 0 */
                        app_ble_set_device_to_connect(ble_device_table[index_for_connected_to_peer].ble_addr);
                        time_wait_to_connect_device_next_start();
                        /* Start timer to scan next device */
                        time_wait_to_connect_device_next_start(); 
                    }
                    else
                    {
                        /* Set g_post_http_start to start POST progress again */
                        g_post_http_start = true;
                    }
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
                    case 2: /* ID 1 MAC */
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
                    case 4: /* ID 2 MAC */
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
                    case 6: /* ID 3 MAC */
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
                    case 8: /* ID 4 MAC */
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
                    case 10: /* ID 5 MAC */
                    {
                        device5[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device5[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device5[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device5[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device5[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device5[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device5[0], device5[1], device5[2], device5[3], device5[4], device5[5]);
                        break;
                    }
                    case 12: /* ID 6 MAC */
                    {
                        device6[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device6[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device6[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device6[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device6[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device6[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device6[0], device6[1], device6[2], device6[3], device6[4], device6[5]);
                        break;
                    }
                    case 14: /* ID 7 MAC */
                    {
                        device7[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device7[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device7[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device7[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device7[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device7[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device7[0], device7[1], device7[2], device7[3], device7[4], device7[5]);
                        break;
                    }
                    case 16: /* ID 8 MAC */
                    {
                        device8[0] = app_convert_char2Dec(data[count + 2], data[count + 3]);
                        device8[1] = app_convert_char2Dec(data[count + 5], data[count + 6]);
                        device8[2] = app_convert_char2Dec(data[count + 8], data[count + 9]);
                        device8[3] = app_convert_char2Dec(data[count + 11], data[count + 12]);
                        device8[4] = app_convert_char2Dec(data[count + 14], data[count + 15]);
                        device8[5] = app_convert_char2Dec(data[count + 17], data[count + 18]);

                        // ESP_LOGI(TAG, "%d %d %d %d %d %d", device8[0], device8[1], device8[2], device8[3], device8[4], device8[5]);
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
    //     ESP_LOGI(TAG_POST, "%d, %c %c", counter, data[counter], data_compare[counter]); 
    // }
    if((data[17] == data_compare[17]) && (data[18] == data_compare[18]) && (data[19] == data_compare[19])) /* Check 200 */
    {
        ESP_LOGI(TAG_POST, "%c%c%c", data[17], data[18], data[19]);
        g_post_http_success = true;
        ESP_LOGW(TAG_POST, "POST to HTTP success");
    }
    else
    {
        retVal = false;
        g_post_http_success = false;
        ESP_LOGE(TAG_POST, "POST to HTTP faile");
    }

    return retVal;
}

/* Check status read all devices */
static uint8_t app_status_read_all_devices(void)
{
    /* Return number devices read avaiable */
    uint8_t retVal = 0; 
    uint8_t count = 0;
    
    for(count = 0; count < MAX_DEVICE_NUM; count ++)
    {
        if(g_status_read_all_device[count])
        {
            retVal += 1;
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

/* Compare MAC id */
static uint8_t app_compare_mac_id(ble_device_inst_t device_info)
{
    uint8_t retVal = 0;

    if((device_info.ble_addr[0] == device1[0]) && (device_info.ble_addr[1] == device1[1]) && (device_info.ble_addr[2] == device1[2]) \
      && (device_info.ble_addr[3] == device1[3]) && (device_info.ble_addr[4] == device1[4]) && (device_info.ble_addr[5] == device1[5]))
    {
        retVal = 0;
    }
    else if((device_info.ble_addr[0] == device2[0]) && (device_info.ble_addr[1] == device2[1]) && (device_info.ble_addr[2] == device2[2]) \
      && (device_info.ble_addr[3] == device2[3]) && (device_info.ble_addr[4] == device2[4]) && (device_info.ble_addr[5] == device2[5]))
    {
        retVal = 1;
    }
    else if((device_info.ble_addr[0] == device3[0]) && (device_info.ble_addr[1] == device3[1]) && (device_info.ble_addr[2] == device3[2]) \
      && (device_info.ble_addr[3] == device3[3]) && (device_info.ble_addr[4] == device3[4]) && (device_info.ble_addr[5] == device3[5]))
    {
        retVal = 2;
    }
    else if((device_info.ble_addr[0] == device4[0]) && (device_info.ble_addr[1] == device4[1]) && (device_info.ble_addr[2] == device4[2]) \
      && (device_info.ble_addr[3] == device4[3]) && (device_info.ble_addr[4] == device4[4]) && (device_info.ble_addr[5] == device4[5]))
    {
        retVal = 3;
    }
    else if((device_info.ble_addr[0] == device5[0]) && (device_info.ble_addr[1] == device5[1]) && (device_info.ble_addr[2] == device5[2]) \
      && (device_info.ble_addr[3] == device5[3]) && (device_info.ble_addr[4] == device5[4]) && (device_info.ble_addr[5] == device5[5]))
    {
        retVal = 4;
    }
    else if((device_info.ble_addr[0] == device6[0]) && (device_info.ble_addr[1] == device6[1]) && (device_info.ble_addr[2] == device6[2]) \
      && (device_info.ble_addr[3] == device6[3]) && (device_info.ble_addr[4] == device6[4]) && (device_info.ble_addr[5] == device6[5]))
    {
        retVal = 5;
    }
    else if((device_info.ble_addr[0] == device7[0]) && (device_info.ble_addr[1] == device7[1]) && (device_info.ble_addr[2] == device7[2]) \
      && (device_info.ble_addr[3] == device7[3]) && (device_info.ble_addr[4] == device7[4]) && (device_info.ble_addr[5] == device7[5]))
    {
        retVal = 6;
    }
    else if((device_info.ble_addr[0] == device8[0]) && (device_info.ble_addr[1] == device8[1]) && (device_info.ble_addr[2] == device8[2]) \
      && (device_info.ble_addr[3] == device8[3]) && (device_info.ble_addr[4] == device8[4]) && (device_info.ble_addr[5] == device8[5]))
    {
        retVal = 7;
    }

    return retVal;
}