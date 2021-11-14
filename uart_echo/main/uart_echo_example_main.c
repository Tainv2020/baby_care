/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/timer.h"
#include "lib_gpio.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */
#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define UART0      0
#define UART1      1
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (1024)
#define DELAY_TIME 50

/* Timer */
#define MAX_TIMER 3
#define TIMER_SIM800_ID 0
TimerHandle_t xTimers[MAX_TIMER];

/* AT command */
uint8_t AT[] = "AT\r\n";
uint8_t AT_NO_RESPOND[] = "ATE0\r\n";
uint8_t AT1[] = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n";
uint8_t AT2[] = "AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n";
uint8_t AT3[] = "AT+SAPBR=1,1\r\n";
uint8_t AT4[] = "AT+SAPBR=2,1\r\n";
uint8_t AT5[] = "AT+HTTPINIT\r\n";
uint8_t AT6[] = "AT+HTTPPARA=\"CID\",1\r\n";
uint8_t AT7[] = "AT+HTTPPARA=\"URL\",\"http://bc-api.gl-sci.com/api/Common/SubmitHistoryData\"\r\n";
uint8_t AT8[] = "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n";
uint8_t AT9[] = "{\"dataLoggerCode\": \"hub00001\",\"deviceCode\": \"C9:AD:7F:93:4C:DE\",\"dataTypeID\": 1,\"dataValue\": 33,\"batteryValue\": 22,\"isWarning\": false,\"securityKey\": \"123456\"}\r\n";
char AT10[500];
uint8_t AT11[] = "AT+HTTPACTION=1\r\n";
uint8_t AT12[] = "AT+HTTPREAD\r\n";
uint8_t AT13[] = "AT+HTTPTERM\r\n";



/* UART functions */
void sim800_init(void);
void sim800_get(void);
void task1(void);

static void echo_task1(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        uart_write_bytes(UART1, (const char *) data, len);
        if(len == 1)
        {
            sim800_init();
        }
        else if(len == 2)
        {
            task1();
        }
    }
}

static void echo_task2(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(UART0, (const char *) data, len);
    }
}

void sim800_init(void)
{
    char body[500];

    sprintf(body, "{\"dataLoggerCode\": \"hub00001\",\"deviceCode\": \"6f:6a:f2:03:ad:7b\",\"dataTypeID\": 1,\"dataValue\": 29,\"batteryValue\": 33,\"isWarning\": false,\"securityKey\": \"123456\"}\r\n");
    
    uart_write_bytes(UART1, (const char *) AT, sizeof(AT));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT1, sizeof(AT1));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT2, sizeof(AT2));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT3, sizeof(AT3));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT4, sizeof(AT4));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT5, sizeof(AT5));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT7, sizeof(AT7));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT8, sizeof(AT8));
    vTaskDelay(DELAY_TIME);
    sprintf(AT10, "AT+HTTPDATA=%d,\"10000\"\r\n", strlen(body));
    uart_write_bytes(UART1, (const char *) AT10, sizeof(AT10));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT9, sizeof(AT9));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT11, sizeof(AT11));
}

void sim800_get(void)
{
    uart_write_bytes(UART1, (const char *) AT1, sizeof(AT1));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT2, sizeof(AT2));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT3, sizeof(AT3));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT4, sizeof(AT4));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT5, sizeof(AT5));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT6, sizeof(AT6));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT7, sizeof(AT7));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT8, sizeof(AT8));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT11, sizeof(AT11));
    vTaskDelay(200);
    uart_write_bytes(UART1, (const char *) AT12, sizeof(AT12));
    vTaskDelay(200);
}

void task1(void)
{
    uart_write_bytes(UART1, (const char *) AT7, sizeof(AT7));
    vTaskDelay(DELAY_TIME);
    sprintf(AT10, "AT+HTTPDATA=%d,\"10000\"\r\n", sizeof(AT9));
    uart_write_bytes(UART1, (const char *) AT10, sizeof(AT10));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT9, sizeof(AT9));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT11, sizeof(AT11));
}

/* Soft timer callback */
void vTimerCallback(TimerHandle_t xTimer)
{
    uint8_t Counter = 0;

    configASSERT( xTimer );
    Counter = ( uint32_t ) pvTimerGetTimerID( xTimer );

    if(Counter == TIMER_SIM800_ID)
    {
        uart_write_bytes(UART0, "init sim800", 11);
        sim800_init();
    }
}

void app_main(void)
{
    lib_gpio_output_init(4);
    lib_gpio_output_init(5);
    lib_gpio_output_init(23);
    lib_gpio_write(4, 0);
    lib_gpio_write(5, 1);
    lib_gpio_write(23, 1);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config1 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, ECHO_TEST_RTS, ECHO_TEST_CTS));

    ESP_ERROR_CHECK(uart_driver_install(UART1, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART1, &uart_config1));
    ESP_ERROR_CHECK(uart_set_pin(UART1, 27, 26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Create timer */
    xTimers[TIMER_SIM800_ID] = xTimerCreate("timer task for init sim800", pdMS_TO_TICKS(8000), pdFALSE, (void *)0, vTimerCallback);

    /* Create task */ 
    xTaskCreate(echo_task1, "uart0_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(echo_task2, "uart1_task", ECHO_TASK_STACK_SIZE, NULL, 11, NULL);

    /* Start timer */ 
    // xTimerStart(xTimers[TIMER_SIM800_ID], 0);
}
