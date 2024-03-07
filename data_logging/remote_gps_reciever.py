import serial
import time
import socket
import pickle

class RemoteGPSReciever:
    def __init__(self, arduino_port='/dev/ttyACM0', baud_rate=9600):
        self.gpsData = []
        self.terminate_flag = False

    def main(self):
        # Open the serial port
        ser = serial.Serial(self.arduino_port, self.baud_rate)

        host = '10.101.180.10'  # Client(drone)'s IP address
        port = 4050  # Port for communication

        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # connect to the server on local computer
        try:
            s.connect((host, port))

            # recieve data from server
            while not self.terminate_flag: # Check terminate flag
                try:
                    data = s.recv(4096)
                    data = pickle.loads(data)
                    print(data)
                except:
                    # Handle errors or disconnects here
                    print("Connection lost.")
                    s.close()
                    break
        except:
            # Handle errors or disconnects here
            print("No Connection.")
            s.close()


        # Once the connection ends, stop/pause all navigation processes
        # and land the drone
        ##############################################################

    def get_data(self):
        return self.gpsData

    def clear_data(self):
        self.gpsData = [
            [0.0, 0.0, 0, 0],
        ]

    def set_terminate_flag(self):
        self.terminate_flag = True

    def test(self):
        while True:
            print("Logging gps data: " + str(self.gpsData))
            time.sleep(2)

if __name__ == "__main__":
    gps_logger = RemoteGPSReciever()
    gps_logger.main()