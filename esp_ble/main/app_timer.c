#include "app_timer.h"
#include "app_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

TimerHandle_t xTimers[NUM_TIMERS];

void app_timer_init(void *callback)
{
    xTimers[0] = xTimerCreate("TimeoutForReadData",                      // Just a text name, not used by the kernel.
                            pdMS_TO_TICKS(10000),    // 1000ms.
                            pdFALSE,                        // The timers will auto-reload themselves when they expire.
                            (void *) 0,                     // Assign each timer a unique id equal to its array index.
                            callback);                // Each timer calls the same callback when it expires.
                                         
    if(xTimers[0] == NULL)
    {
        // The timer was not created.
    }  

    xTimers[1] = xTimerCreate("TimeWaitToConnect",                      // Just a text name, not used by the kernel.
                            pdMS_TO_TICKS(10000),    // 1000ms.
                            pdFALSE,                        // The timers will auto-reload themselves when they expire.
                            (void *) 1,                     // Assign each timer a unique id equal to its array index.
                            callback);                // Each timer calls the same callback when it expires.
                                         
    if(xTimers[1] == NULL)
    {
        // The timer was not created.
    }

    xTimers[2] = xTimerCreate("TimeWaitToGetDataFromHTTP",                      // Just a text name, not used by the kernel.
                            pdMS_TO_TICKS(5000),    // 1000ms.
                            pdFALSE,                        // The timers will auto-reload themselves when they expire.
                            (void *) 2,                     // Assign each timer a unique id equal to its array index.
                            callback);                // Each timer calls the same callback when it expires.
                                         
    if(xTimers[2] == NULL)
    {
        // The timer was not created.
    }
}


void timeout_for_read_data_start(void)
{
    if(xTimerStart( xTimers[0], 0) != pdPASS )
    {
        // The timer could not be set into the Active state.
    }
}

void timeout_for_read_data_stop(void)
{
    if(xTimerStop( xTimers[0], 0) != pdPASS )
    {
        // The timer could not be set into the Active state.
    }
}

void time_wait_to_connect_device_next_start(void)
{
    if(xTimerStart( xTimers[1], 0) != pdPASS )
    {
        // The timer could not be set into the Active state.
    }
}

void time_stop_to_connect_device_next_start(void)
{
    if(xTimerStop( xTimers[1], 0) != pdPASS )
    {
        // The timer could not be set into the Active state.
    }
}

void timer_change_period_and_start(int index, int ms)
{
    xTimerChangePeriod(xTimers[index],ms/portTICK_PERIOD_MS, 0);
}

/* Timeout to start GET datafrom HTTP */
void timeout_for_get_data_from_http_start(void)
{
    if(xTimerStart( xTimers[2], 0) != pdPASS )
    {
        // The timer could not be set into the Active state.
    }
}