from data_logging.onboard_gps_logger import OnboardGPSLogger
from remoteBase.data_logging.remote_gps_logger import RemoteGPSLogger
from data_logging.odometry import Odometry
import multiprocessing
import time
import signal

# Global flag to indicate whether to terminate processes
terminate_flag = False

# Create a list of waypoints
waypoints = [
    [34.0675, -118.4505],  # Los Angeles
    [34.0522, -118.2437],  # Hollywood
    [34.0522, -118.2437],  # Santa Monica
    [34.0522, -118.2437],  # Malibu
]


# Handle Ctrl+C
def signal_handler(sig, frame):
    print('Termination signal received! Terminating processes...')
    terminate_flag = True


def main():
    global terminate_flag

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("Starting Waypoint Navigation")

    # Start the GPS and Odometry processes
    onboard_gps_logger = OnboardGPSLogger()
    odometry = Odometry()

    gps_process = multiprocessing.Process(target=onboard_gps_logger.test)
    gps_process.start()
    
    odometry_process = multiprocessing.Process(target=odometry.test)
    odometry_process.start()

    # Loop through the waypoints until the terminate flag is set
    while not terminate_flag:
        # -------------------------------------

        time.sleep(1)

        # -------------------------------------
    

    # Set the terminate flag for each process
    onboard_gps_logger.set_terminate_flag()
    odometry.set_terminate_flag()

    print("Waypoint Navigation terminated!")


if __name__ == '__main__':
    main()