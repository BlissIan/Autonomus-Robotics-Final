from gpiozero import Motor, PWMOutputDevice
from multiprocessing import Process, Queue
from queue import Empty
import time
import math


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

    #Implement linear interpolation to bound to a percent duty cycle
    wR_percent = ((wR - (-bound)) / (bound -(-bound)) * (output_bound - (-output_bound)) + (-output_bound))
    wL_percent = ((wL - (-bound)) / (bound -(-bound)) * (output_bound - (-output_bound)) + (-output_bound))

    wR_percent = round(wR_percent,2)
    wL_percent = round(wL_percent,2)

    #print(f"Right Wheel = {wR_percent} Left Wheel = {wL_percent}")

    right_side(wR_percent)
    left_side(wL_percent)

def Cord_interp(cords):
    for angle, distance in cords:

        # Ignore far points
        if distance > 200:
            continue

        # Normalize angle to 0–2π
        a = angle % (2 * math.pi)

        # FRONT
        if a < math.radians(45) or a > math.radians(315):
            print("Obstacle in FRONT")
            print(f"Angle: {math.degrees(a):.2f} degrees, Distance: {distance:.2f} cm")

        # RIGHT
        if math.radians(45) <= a < math.radians(135):
            print("Obstacle on RIGHT")
            print(f"Angle: {math.degrees(a):.2f} degrees, Distance: {distance:.2f} cm")
        # BACK
        #if math.radians(135) <= a < math.radians(225):
        #    print("Obstacle in BACK")

        # LEFT
        if math.radians(225) <= a < math.radians(315):
            print("Obstacle on LEFT")
            print(f"Angle: {math.degrees(a):.2f} degrees, Distance: {distance:.2f} cm")


def Rover_control(queue1, stop_event):
    print("Driving thread started")

    try:
        while not stop_event.is_set():

            # Non-blocking queue read
            try:
                cords = queue1.get_nowait()
            except Empty:
                cords = None

            # Process LIDAR points if available
            if cords:
                Cord_interp(cords)

            # Rover logic runs every loop
            drive(0, 0)

            time.sleep(0.01)

    finally:
        print("Stopping rover")
        drive(0, 0)

        right_pwm.close()
        left_pwm.close()
        right_motor.close()
        left_motor.close()

