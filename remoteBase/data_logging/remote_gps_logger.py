import serial
import time
import socket
import pickle

class RemoteGPSLogger:
    def __init__(self, terminate_flag, gpsData, arduino_port, baud_rate, host, port):
        self.terminate_flag = terminate_flag
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.host = host
        self.port = port
        self.gpsData = gpsData

    def main(self):
        print("*  Starting Remote GPS Logger...")

        # Open the serial port
        ser = serial.Serial(self.arduino_port, self.baud_rate)

        #self.HOST = '10.101.180.10' # Server(remoteBase)'s IP address
        #self.PORT = 4050  # Port for communication

        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # bind the server to the host and port
        s.bind((self.host, self.port))
        s.listen(2)

        # accept connection
        print("Waiting for connection...")
        conn, addr = s.accept()
        print("Connected to: " + str(addr))

        # get data from arduino serial
        while not self.terminate_flag.value: # Check terminate flag
            data = ser.readline().decode('utf-8')
            data = data.split(',')
            latitude = float(data[0])
            longitude = float(data[1])
            timestamp = int(data[2])
            satellites = int(data[3])
            data = [longitude, latitude, timestamp, satellites]
            print(data)

            # log the new gps data
            self.gpsData.append(data)

            # send data from remote base to drone
            conn.send(pickle.dumps(data))
            print("Data sent: " + str(data))

        # close the connection
        conn.close()
        s.close()

        print("*  Remote GPS Logger terminated!")

    def get_data(self):
        return self.gpsData

    def clear_data(self):
        self.gpsData = [
            [0.0, 0.0, 0, 0],
        ]

    # TESTING
    def test(self):
        print("*  TEST: Starting Remote GPS Logger...")

        try:
            while not self.terminate_flag.value:
                print("Logging gps data: " + str(self.gpsData))
                time.sleep(2)
        except Exception as e:
            print("Error in Remote GPS Logger TEST: " + str(e))

        print("*  TEST: Remote GPS Logger terminated!")

if __name__ == "__main__":
    gps_logger = RemoteGPSLogger()
    gps_logger.main()