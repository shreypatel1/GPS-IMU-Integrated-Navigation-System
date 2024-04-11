import multiprocessing
import threading
import keyboard
import time
import djitellopy
from data_logging.imu_logger import IMU_Logger
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
    imuData = manager.list([[0.0, 0.0, 0, 0]]) # [x, y, yaw, timestamp]
    onboardGPSData = manager.list([[0.0, 0.0, 0, 0]]) # [longitude, latitude, timestamp, satellites]
    #remoteGPSData = manager.list([[-84.5214510, 33.9370979, 0, 12]])
    remoteGPSData = [
        [-84.5214510, 33.9370979, 0, 12],
    ]
    odometryData = manager.list([[0.0, 0.0, 0, 0]])

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    # Tello drone object
    tello = djitellopy.Tello()
    tello.connect(wait_for_state=True)

    print("Starting Flight Navigation...")


    # Instantiate the classes
    imu_logger = IMU_Logger(terminate_flag, tello, imuData) # This logs the IMU data
    onboard_gps_logger = OnboardGPSLogger(terminate_flag, onboardGPSData, '/dev/ttyACM0', 9600) # This logs the onboard GPS data
    #remote_gps_reciever = RemoteGPSReciever(terminate_flag, remoteGPSData, '10.101.180.10', 4050) # This recieves the remote GPS data
    odometry = Odometry(terminate_flag, odometryData) # This calculates the drone odometry


    # Start the subprocesses
    imu_process = multiprocessing.Process(target=imu_logger.main)
    imu_process.start()

    oGPS_process = multiprocessing.Process(target=onboard_gps_logger.main)
    oGPS_process.start()

    #rGPS_process = multiprocessing.Process(target=remote_gps_reciever.main)
    #rGPS_process.start()
    
    odometry_process = multiprocessing.Process(target=odometry.main)
    odometry_process.start()


    tello.takeoff()


    # Loop through the waypoints until the terminate flag is set
    while not terminate_flag.value:
        # -------------------------------------
        
        # Get the current GPS data
        # Control the drone to its target location using the PID controller
        time.sleep(5)

        # -------------------------------------
    

    print("Terminating Flight Navigation...")

    # Wait for the processes to terminate
    oGPS_process.join()
    #rGPS_process.join()
    odometry_process.join()

    print("Flight Navigation terminated!")


if __name__ == '__main__':
    main()