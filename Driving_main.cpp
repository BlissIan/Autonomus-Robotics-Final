#include <stdio.h>
#include <lgpio.h>
#include <unistd.h>
#include "gpio_control.h"


//pwm pins 26 and 23
void init_motors(int handle){

    // RIGHT MOTOR
    gpio_set_output(handle, 6);
    gpio_set_output(handle, 5);
    gpio_set_output(handle, 26);

    // LEFT MOTOR
    gpio_set_output(handle, 25);
    gpio_set_output(handle, 24);
    gpio_set_output(handle, 23);

}

void Drive_forward(int handle, int speed){
    //set motors to go forward
    gpio_write(handle, 5, 0);
    gpio_write(handle, 24, 0);

    gpio_write(handle, 6, 1);
    gpio_write(handle, 25, 1);

    //set motor speed
    gpio_pwm(handle,26, speed);
    gpio_pwm(handle,23, speed);

}

void shutdown_motors(int handle){
    // Stop PWM
    lgTxPwm(handle, 26, 0, 0.0, 0, 0);
    lgTxPwm(handle, 23, 0, 0.0, 0, 0);

    //Stop motors
    lgGpioWrite(handle, 5, 0);
    lgGpioWrite(handle, 6, 0);
    lgGpioWrite(handle, 25, 0);
    lgGpioWrite(handle, 24, 0);
    
    
}

int main()
{
    int handle = lgGpiochipOpen(0);
    init_motors(handle);

    Drive_forward(handle, 50);
    
    usleep(5000000); // 500 ms


    shutdown_motors(handle);
    gpio_close(handle);

    return 0;
}
