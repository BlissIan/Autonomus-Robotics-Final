#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

// Initialize GPIO chip and return handle
int gpio_init();

// Set pin as output
int gpio_set_output(int handle, int pin);

// Write gpio
int gpio_write(int handle, int pin, int value);

//write pwm
int gpio_pwm(int handle, int, int value);

// Close GPIO
void gpio_close(int handle);

#endif
