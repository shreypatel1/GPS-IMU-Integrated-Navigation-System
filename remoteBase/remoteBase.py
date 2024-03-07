from remoteBase.data_logging.remote_gps_logger import RemoteGPSLogger
import multiprocessing
import time
import signal

# Global flag to indicate whether to terminate processes
terminate_flag = False


# Handle Ctrl+C
def signal_handler(sig, frame):
    print('Termination signal received! Terminating processes...')
    terminate_flag = True


def main():
    global terminate_flag

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("Starting Remote Base")

    # Start the Remote logger processes
    remote_gps_logger = RemoteGPSLogger()

    gps_process = multiprocessing.Process(target=remote_gps_logger.test)
    gps_process.start()

    # Wait for the termination flag
    while not terminate_flag:
        # -------------------------------------
        time.sleep(1)
        # -------------------------------------
    

    # Set the terminate flag for each process
    remote_gps_logger.set_terminate_flag()

    print("Remote Base terminated!")


if __name__ == '__main__':
    main()