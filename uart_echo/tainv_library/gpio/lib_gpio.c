#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "lib_gpio.h"

lib_gpio_callback_func_t call_back_func = NULL;

/* Function handle interrupt event */
static void IRAM_ATTR lib_gpio_input_handler(void *arg)
{
   int pin = (uint32_t) arg;
   call_back_func(pin);
}

void lib_gpio_input_init(gpio_num_t pin, gpio_pull_mode_t pull_type, gpio_int_type_t interrupt_type)
{
   gpio_pad_select_gpio(pin);
   gpio_set_direction(pin, GPIO_MODE_INPUT);
   gpio_set_pull_mode(pin, pull_type);
   gpio_set_intr_type(pin, interrupt_type);
   gpio_install_isr_service(0);
   gpio_isr_handler_add(pin, lib_gpio_input_handler, (void *)pin); /* Register interrupt function */
}

void lib_gpio_output_init(gpio_num_t pin)
{
   gpio_pad_select_gpio(pin);
   gpio_set_direction(pin, GPIO_MODE_OUTPUT);
   gpio_set_intr_type(pin, GPIO_INTR_DISABLE);
}

void lib_gpio_write(gpio_num_t pin, uint32_t level)
{
   gpio_set_level(pin, level);
}

int lib_gpio_read(gpio_num_t pin)
{
   return gpio_get_level(pin);
}

void lib_gpio_toggled(gpio_num_t pin)
{
   uint32_t value = gpio_get_level(pin);

   gpio_set_level(pin, !value);
}

void lib_gpio_set_callback(void *func)
{
   call_back_func = func;
}