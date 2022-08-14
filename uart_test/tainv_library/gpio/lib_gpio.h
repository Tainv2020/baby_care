#ifndef __LIB_GPIO_H__
#define __LIB_GPIO_H__

typedef void (*lib_gpio_callback_func_t) (int);

void lib_gpio_input_init(gpio_num_t pin, gpio_pull_mode_t pull_type, gpio_int_type_t interrupt_type);

void lib_gpio_output_init(gpio_num_t pin);

void lib_gpio_write(gpio_num_t pin, uint32_t level);

int lib_gpio_read(gpio_num_t pin);

void lib_gpio_toggled(gpio_num_t pin);

void lib_gpio_set_callback(void *func);

#endif /* __LIB_GPIO_H__ */