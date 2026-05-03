#include <stdio.h>
#include <lgpio.h>
#include <unistd.h>
#include <iostream>
#include "gpio_control.h"

int gpio_init()
{
    int handle = lgGpiochipOpen(0);  // Open gpiochip0
    if (handle < 0)
    {
        std::cerr << "Failed to open gpiochip\n";
    }
    return handle;
}

int gpio_set_output(int handle, int pin)
{
    int status =lgGpioClaimOutput(handle, 0, pin, 0);

    if (status < 0)
    {
        std::cerr << "Failed to set pin as output\n";
    }
    return status;
}

int gpio_write(int handle, int pin, int value){
    int status = lgGpioWrite(handle, pin, value);

     if (status < 0)
    {
        std::cerr << "Failed to write to pin\n";
    }
    return status;

}

int gpio_pwm(int handle, int pin, int value){
    // Start PWM value 0 - 100
    lgTxPwm(handle, pin, 1000, value, 0, 0);  // 1kHz, 50%, continuous

    return(0);
}



void gpio_close(int handle)
{
    lgGpiochipClose(handle);
}

