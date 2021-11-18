#ifndef __APP_TIMER_H
#define __APP_TIMER_H
#include <stdio.h>
#include <stdint.h>
#define NUM_TIMERS 4
void app_timer_init(void *callback);
void timeout_for_read_data_start(void);
void timeout_for_read_data_stop(void);
void time_wait_to_connect_device_next_start(void);
void time_stop_to_connect_device_next_start(void);
void timer_change_period_and_start(int index, int ms);
/* Timeout to start init sim800 */
void timeout_for_start_init_sim800(void);
/* Timeout to start GET datafrom HTTP */
void timeout_for_get_data_from_http_start(void);
#endif