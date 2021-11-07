#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "app_uart.h"

static const char *TAG = "uart_events";
static const char *TAG_UART_SIM800 = "SIM800";

#define EX_UART_NUM 0

#define BUF_SIZE (1024)
#define RX_BUF_SIZE (BUF_SIZE)

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define DELAY_TIME 100

/* AT command */
uint8_t AT[] = "AT\r\n";
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
uint8_t AT_NO_RESPOND[] = "ATE0\r\n";

static QueueHandle_t uart0_queue;
static uint8_t uart_rx_buf[RX_BUF_SIZE];
static app_uart_data_cb_t app_uart_data_cb = NULL;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, uart_rx_buf, event.size, portMAX_DELAY);
                    app_uart_data_cb(uart_rx_buf, (uint16_t)event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
}

void app_uart_init(void)
{
        /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, 22, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}


void app_uart_set_data_callback(app_uart_data_cb_t cb)
{
    app_uart_data_cb = cb;
}

static void uart1_event_task(void *arg)
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

void app_uart1_sim800_init(uint8_t uart_instance, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin, uint8_t priority)
{
    int intr_alloc_flags = 0;
    uart_config_t uart_config = {
    .baud_rate = baudrate,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
    };
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uart_instance, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart_instance, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_instance, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if(UART1 == uart_instance)
    {
        xTaskCreate(uart1_event_task, "uart1_task", 2048, NULL, priority, NULL);
    }
    
}

/* Init sim800 */ 
void app_uart_sim800_init(void)
{
    uart_write_bytes(UART1, (const char *) AT, sizeof(AT));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT_NO_RESPOND, sizeof(AT_NO_RESPOND));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT1, sizeof(AT1));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT2, sizeof(AT2));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT3, sizeof(AT3));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT4, sizeof(AT4));
    vTaskDelay(DELAY_TIME);
}

/* POST data to sever */
void app_uart_post(uint8_t temp, uint8_t battery)
{
    char body[500];
    sprintf(body, "{\"dataLoggerCode\": \"hub00001\",\"deviceCode\": \"C9:AD:7F:93:4C:DE\",\"dataTypeID\": 1,\"dataValue\": %d,\"batteryValue\": %d,\"isWarning\": false,\"securityKey\": \"123456\"}\r\n", temp, battery);

    uart_write_bytes(UART1, (const char *) AT5, sizeof(AT5));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT6, sizeof(AT6));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT7, sizeof(AT7));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT8, sizeof(AT8));
    vTaskDelay(DELAY_TIME);
    sprintf(AT10, "AT+HTTPDATA=%d,\"10000\"\r\n", strlen(body));
    uart_write_bytes(UART1, (const char *) AT10, sizeof(AT10));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) body, sizeof(body));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT11, sizeof(AT11));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT12, sizeof(AT12));
    vTaskDelay(DELAY_TIME);
    uart_write_bytes(UART1, (const char *) AT13, sizeof(AT13));
}