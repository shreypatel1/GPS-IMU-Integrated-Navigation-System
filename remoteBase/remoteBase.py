import multiprocessing
import time
import keyboard
import threading
from data_logging.remote_gps_logger import RemoteGPSLogger

# Global flag to indicate whether to terminate processes
terminate_flag = None


# Handle Ctrl+C
def termination_listener():
    global terminate_flag
    keyboard.wait('esc')
    print('Termination signal received! Terminating processes...')
    terminate_flag.value = True


def main():
    global terminate_flag

    # Create a manager for shared variables
    manager = multiprocessing.Manager()
    terminate_flag = manager.Value('b', False)
    gpsData = manager.list([[0.0, 0.0, 0, 0]])

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    print("Starting Remote Base...")

    # Start the Remote logger processes
    remote_gps_logger = RemoteGPSLogger(terminate_flag, gpsData, '/dev/ttyACM0', 9600, '10.101.180.10', 4050) # This logs the remote GPS data

    gps_process = multiprocessing.Process(target=remote_gps_logger.test)
    gps_process.start()


    # Wait for the termination flag
    while not terminate_flag.value:
        # -------------------------------------
        time.sleep(1)
        # -------------------------------------
    

    # Wait for the processes to terminate
    gps_process.join()

    print("Remote Base terminated!")


if __name__ == '__main__':
    main()