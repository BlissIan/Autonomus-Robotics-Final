from threading import Thread
from multiprocessing import Queue
import time
from Lidar import stop_event
from Lidar import Lidar_Scan, stop_event
from Driving import Rover_control


if __name__ == "__main__":
    q1 = Queue()

   

    Rover_drive_thread = Thread(target=Rover_control, args=(q1, stop_event))
    Lidar_thread = Thread(target=Lidar_Scan)

    try:
        Rover_drive_thread.start()
        Lidar_thread.start()

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down...")
        stop_event.set()

        Rover_drive_thread.join()
        Lidar_thread.join()
