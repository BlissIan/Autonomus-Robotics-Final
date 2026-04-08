from gpiozero import Motor, PWMOutputDevice
from multiprocessing import Process, Queue
import time



right_motor = Motor(forward=6, backward=5)
right_pwm = PWMOutputDevice(26)
right_pwm.value = 0

left_motor = Motor(forward=25, backward=24)
left_pwm = PWMOutputDevice(23)
left_pwm.value = 0

#Right side speed function
def right_side(speed):
    if speed > 0:
        right_motor.forward()
        right_pwm.value = abs(speed)
    elif speed < 0:
        right_motor.backward()
        right_pwm.value = abs(speed)
    else:
        right_pwm.value = abs(speed)


#Left side speed function
def left_side(speed):
    if speed > 0:
        left_motor.forward()
        left_pwm.value = abs(speed)
    elif speed < 0:
        left_motor.backward()
        left_pwm.value = abs(speed)
    else:
        left_pwm.value = abs(speed)


def saturate(value, upper, lower):
    return max(lower, min(value, upper))


#Driving using inverse kinamatics
def drive(w, v):
    r = 0.03175
    L = 0.13335

    wL = (v - (L* w / 2)) *1/r
    wR = (v + (L* w / 2)) *1/r

    
    bound = 2
    output_bound = 1

    #Bound left wheel
    if wL > bound:
        wL = bound
    elif wL < -bound:
        wL = -bound

    #Bound Right wheel
    if wR > bound:
        wR = bound
    elif wR < -bound:
        wR = -bound

    #Implement linear interpolation to bould to a percent duty cycle
    wR_percent = ((wR - (-bound)) / (bound -(-bound)) * (output_bound - (-output_bound)) + (-output_bound))
    wL_percent = ((wL - (-bound)) / (bound -(-bound)) * (output_bound - (-output_bound)) + (-output_bound))

    wR_percent = round(wR_percent,2)
    wL_percent = round(wL_percent,2)

    #print(f"Right Wheel = {wR_percent} Left Wheel = {wL_percent}")

    right_side(wR_percent)
    left_side(wL_percent)



#Driving logic
def Rover_control(queue1, stop_event):
    try:
        while not stop_event.is_set():
            drive(0,1)
            time.sleep(0.1)

    finally:
        print("Stopping rover")
        drive(0,0)

        right_pwm.close()
        left_pwm.close()
        right_motor.close()
        left_motor.close()


