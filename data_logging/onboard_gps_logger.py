import serial
import time

class OnboardGPSLogger:
    def __init__(self, terminate_flag, onboardGPSData, arduino_port, baud_rate):
        self.terminate_flag = terminate_flag
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.gpsData = onboardGPSData

    def main(self):
        print("*  Starting Onboard GPS Logger...")

        try:
            # Open the serial port
            ser = serial.Serial(self.arduino_port, self.baud_rate)

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
        except Exception as e:
            print("Error in Onboard GPS Logger: " + str(e))
        
        print("*  Onboard GPS Logger terminated!")

    def get_data(self):
        return self.gpsData

    def clear_data(self):
        self.gpsData = [
            [0.0, 0.0, 0, 0],
        ]

    # TESTING
    def test(self):
        print("*  TEST: Starting Onboard GPS Logger...")

        try:
            while not self.terminate_flag.value:
                #print("Logging gps data: " + str(self.gpsData))
                time.sleep(2)
        except Exception as e:
            print("Error in Onboard GPS Logger TEST: " + str(e))

        print("*  TEST: Onboard GPS Logger terminated!")

if __name__ == "__main__":
    gps_logger = OnboardGPSLogger()
    gps_logger.main()