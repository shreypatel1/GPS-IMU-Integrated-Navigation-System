import multiprocessing
import threading
import keyboard
import time
from data_logging.onboard_gps_logger import OnboardGPSLogger
from data_logging.remote_gps_reciever import RemoteGPSReciever
from data_logging.odometry import Odometry


# Multiprocess variables
terminate_flag = None
onboardGPSData = None
remoteGPSData = None
odometryData = None

# Create a list of waypoints
waypoints = [
    [34.0675, -118.4505],  # Los Angeles
    [34.0522, -118.2437],  # Hollywood
    [34.0522, -118.2437],  # Santa Monica
    [34.0522, -118.2437],  # Malibu
]


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
    onboardGPSData = manager.list([[0.0, 0.0, 0, 0]])
    remoteGPSData = manager.list([[0.0, 0.0, 0, 0]])
    odometryData = manager.list([[0.0, 0.0, 0, 0]])

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    print("Starting Flight Navigation...")

    # Start the subprocesses
    onboard_gps_logger = OnboardGPSLogger(terminate_flag, onboardGPSData, '/dev/ttyACM0', 9600) # This logs the onboard GPS data
    remote_gps_reciever = RemoteGPSReciever(terminate_flag, remoteGPSData, '10.101.180.10', 4050) # This recieves the remote GPS data
    odometry = Odometry(terminate_flag, odometryData) # This calculates the drone odometry

    oGPS_process = multiprocessing.Process(target=onboard_gps_logger.test)
    oGPS_process.start()

    rGPS_process = multiprocessing.Process(target=remote_gps_reciever.test)
    rGPS_process.start()
    
    odometry_process = multiprocessing.Process(target=odometry.test)
    odometry_process.start()


    # Loop through the waypoints until the terminate flag is set
    while not terminate_flag.value:
        # -------------------------------------
        
        print("Odometry data in main: " + str(odometryData))
        time.sleep(5)

        # -------------------------------------
    

    print("Terminating Flight Navigation...")

    # Wait for the processes to terminate
    oGPS_process.join()
    rGPS_process.join()
    odometry_process.join()

    print("Flight Navigation terminated!")


if __name__ == '__main__':
    main()