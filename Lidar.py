import math
import time
import numpy as np
import threading
from rplidar import RPLidar

# -------------------------------
# Setup
# -------------------------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 256000
lidar = RPLidar(PORT, baudrate=BAUDRATE)

stop_event = threading.Event()
latest_scan = []  
latest_scan_lock = threading.Lock()
#running = True    

# -------------------------------
# Thread to continuously read scans
# -------------------------------
def read_lidar():
    try:
        for scan in lidar.iter_scans(max_buf_meas=5000):
            if stop_event.is_set():
                break
            with latest_scan_lock:
                latest_scan[:] = scan
    except Exception as e:
        print("LIDAR read error:", e)

# -------------------------------
# Helper functions
# -------------------------------
def get_points_polar():
    points = []
    with latest_scan_lock:
        scan_copy = latest_scan.copy()
    for _, angle, distance in scan_copy:
        if distance > 0:
            points.append((np.round(math.radians(angle), 2), np.round(distance, 2)))
    return points

def get_points_cartesian():
    points = []
    with latest_scan_lock:
        scan_copy = latest_scan.copy()
    for _, angle, distance in scan_copy:
        if distance > 0:
            theta = math.radians(angle)
            x = distance * math.cos(theta)
            y = distance * math.sin(theta)
            points.append((np.round(x, 2), np.round(y, 2)))
    return points

# -------------------------------
# Main loop
# -------------------------------
def Lidar_Scan():
    reader_thread = threading.Thread(target=read_lidar)
    reader_thread.start()

    try:
        while not stop_event.is_set():
            polar_points = get_points_polar()

            if polar_points:
                print(f"length: {len(polar_points)}")

            time.sleep(0.05)

    finally:
        print("Stopping LIDAR...")

        stop_event.set()

        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()

        reader_thread.join(timeout=2)

        print("Disconnected cleanly.")
