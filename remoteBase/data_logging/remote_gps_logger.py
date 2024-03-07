import serial
import time
import socket
import pickle

class RemoteGPSLogger:
    def __init__(self, arduino_port='/dev/ttyACM0', baud_rate=9600):
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.gpsData = []
        self.terminate_flag = False

    def main(self):
        # Open the serial port
        ser = serial.Serial(self.arduino_port, self.baud_rate)

        host = '10.101.180.10'  # server(remoteBase)'s IP address
        port = 4050  # Port for communication

        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # bind the server to the host and port
        s.bind((host, port))
        s.listen(1)

        # accept connection
        print("Waiting for connection...")
        conn, addr = s.accept()
        print("Connected to: " + str(addr))

        # get data from arduino serial
        while not self.terminate_flag: # Check terminate flag
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
    gps_logger = RemoteGPSLogger()
    gps_logger.main()