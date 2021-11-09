#ifndef APP_UART_H
#define APP_UART_H
#include <stdint.h>

#define UART0                   0
#define UART1                   1
#define TXD_UART0_PIN           22
#define RXD_UART0_PIN           23
#define TXD_UART1_PIN           27
#define RXD_UART1_PIN           26

typedef void (*app_uart_data_cb_t) (uint8_t,uint8_t*,uint16_t);

void app_uart_init(void);
void app_uart_set_data_callback(app_uart_data_cb_t cb);
void app_uart1_sim800_init(uint8_t uart_instance, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin, uint8_t priority);
/* Init sim800 */
void app_uart_sim800_init(void);
/* POST data to server */
void app_uart_post(uint8_t temp, uint8_t battery);
/* GET data from server */
void app_uart_get(void);

#endif