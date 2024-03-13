import serial
import time
import socket
import pickle

class RemoteGPSReciever:
    def __init__(self, terminate_flag, remoteGPSData, host, port):
        self.terminate_flag = terminate_flag
        self.host = host
        self.port = port
        self.gpsData = remoteGPSData

    def main(self):
        print("*  Starting Remote GPS Reciever...")

        # self.HOST = '10.101.180.10' # Client(drone)'s IP address
        # self.PORT = 4050 # Port for communication

        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # connect to the server on local computer
        try:
            s.connect((self.host, self.port))

            # recieve data from server
            while not self.terminate_flag.value: # Check terminate flag
                try:
                    data = s.recv(4096)
                    data = pickle.loads(data)
                    print(data)
                    self.gpsData.append(data)
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
        
        print("*  Remote GPS Reciever terminated!")

    def get_data(self):
        return self.gpsData

    def clear_data(self):
        self.gpsData = [
            [0.0, 0.0, 0, 0],
        ]

    # TESTING
    def test(self):
        print("*  TEST: Starting Remote GPS Reciever...")

        try:
            while not self.terminate_flag.value:
                #print("Recieving gps data: " + str(self.gpsData))
                time.sleep(2)
        except Exception as e:
            print("Error in Remote GPS Reciever TEST: " + str(e))

        print("*  TEST: Remote GPS Reciever terminated!")

if __name__ == "__main__":
    gps_logger = RemoteGPSReciever()
    gps_logger.main()