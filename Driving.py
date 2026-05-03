from gpiozero import Motor, PWMOutputDevice
from multiprocessing import Process, Queue
from queue import Empty
import time
import math
import numpy as np

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
    L = 0.13

    wL = (v - (L* w / 2)) 
    wR = (v + (L* w / 2)) 

    
    bound = 1
    output_bound = 1
    max_speed = 1.0

    wL = saturate(wL, max_speed, -max_speed)
    wR = saturate(wR, max_speed, -max_speed)

    #Implement linear interpolation to bound to a percent duty cycle
    wR_percent = ((wR - (-bound)) / (bound -(-bound)) * (output_bound - (-output_bound)) + (-output_bound))
    wL_percent = ((wL - (-bound)) / (bound -(-bound)) * (output_bound - (-output_bound)) + (-output_bound))

    wR_percent = round(wR_percent,2)
    wL_percent = round(wL_percent,2)

    print(f"Right Wheel = {wR_percent} Left Wheel = {wL_percent}")

    right_side(wR_percent)
    left_side(wL_percent)

def Cord_interp(cords):
    front = None
    right = None
    left = None

    for angle, distance in cords:

        if distance > 800:
            continue

        a = angle % (2 * math.pi)

        # FRONT
        if a < math.radians(45) or a > math.radians(315):
            if front is None or distance < front:
                front = distance

        # RIGHT
        elif math.radians(45) <= a < math.radians(135):
            if right is None or distance < right:
                right = distance

        # LEFT
        elif math.radians(225) <= a < math.radians(315):
            if left is None or distance < left:
                left = distance

    return [front, right, left]

        
def turn_in_place(angle_deg, speed = 0.1):
    r = 0.03175
    L = 0.13

    angle_rad = np.deg2rad(angle_deg)

    Wr = speed * L / (2/r)
    Wl = -Wr

    if angle_rad <0:
        Wr,Wl = -Wr,-Wl

    v = (r/2) *(Wr + Wl)
    w = (r/2) * (Wr - Wl)

    return np.array([v, w])


def Rover_control(queue1, stop_event):
    print("Driving thread started")

    state = "SEARCH"
    prev_error = 0

    last_cords = None
    last_update_time = time.time()
    timeout = 0.5  # seconds

    try:
        while not stop_event.is_set():

            # ===== GET DATA (non-blocking) =====
            try:
                new_cords = queue1.get_nowait()
                last_cords = new_cords
                last_update_time = time.time()
            except Empty:
                pass

            # ===== SAFETY: no data =====
            if last_cords is None:
                print("No sensor data → stopping")
                drive(0, 0)
                time.sleep(0.01)
                continue

            cords = last_cords

            # ===== TIMEOUT SAFETY =====
            if time.time() - last_update_time > timeout:
                print("Sensor timeout → stopping")
                drive(0, 0)
                time.sleep(0.01)
                continue

            front_dist, right_dist, left_dist = Cord_interp(cords)

            # ===== PARAMETERS =====
            d_ref = 400
            front_limit = 500
            left_thresh = 400
            speed = 0.4

            kp = 0.01
            kd = 0.005

            print(state, front_dist, left_dist, right_dist)

            # ===== STATE MACHINE =====

            # ---------------- SEARCH ----------------
            if state == "SEARCH":

                # valid wall detection (NOT just non-None)
                if left_dist is not None and left_dist < 1200:
                    state = "FOLLOW"

                elif front_dist is not None and front_dist < front_limit:
                    state = "TURN_RIGHT_CORNER"

                else:
                    # slow search
                    drive(0, speed)


            # ---------------- FOLLOW ----------------
            elif state == "FOLLOW":

                # lost wall → search again
                if left_dist is None:
                    state = "SEARCH"

                # obstacle ahead → turn
                elif front_dist is not None and front_dist < front_limit:
                    state = "TURN_RIGHT"

                else:
                    print("following now")
                    error = d_ref - left_dist
                    d_term = error - prev_error
                    prev_error = error

                    w = -kp * error - kd * d_term
                    w = saturate(w, 7, -7)

                    drive(w, speed)


            # ---------------- TURN RIGHT ----------------
            elif state == "TURN_RIGHT":

                drive(-8.5, 0)

                # exit condition (must be stable)
                if (front_dist is None or front_dist > front_limit) or (left_dist is not None and left_dist > left_thresh):
                    time.sleep(0.02)
                    state = "FOLLOW"


            # ---------------- CORNER TURN ----------------
            elif state == "TURN_RIGHT_CORNER":

                drive(-8.5, 0)

                # only exit when space is clear
                if (front_dist is None or front_dist > front_limit) or (left_dist is not None and left_dist > left_thresh):
                    time.sleep(0.2)
                    state = "FOLLOW"
                


            time.sleep(0.01)

    finally:
        print("Stopping rover")
        drive(0, 0)

        right_pwm.close()
        left_pwm.close()
        right_motor.close()
        left_motor.close()


