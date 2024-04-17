import time
import math

class Odometry:
    def __init__(self, terminate_flag, update_imuPos, odometryData, imuData, onboardGPSData):
        self.terminate_flag = terminate_flag
        self.x = 0.0
        self.y = 0.0
        self.theta = 0
        self.timestamp = 0
        self.update_imuPos = update_imuPos
        self.odometryData = odometryData
        self.imuData = imuData
        self.onboardGPSData = onboardGPSData

    def update(self, x, y, theta, timestamp):
        self.x = x
        self.y = y
        self.theta = theta
        self.timestamp = timestamp
        self.odometryData.append([self.x, self.y, self.theta, self.timestamp])

    def get(self):
        return self.x, self.y, self.theta, self.timestamp
    
    def main(self):
        print("*  Starting Odometry Calculations...")

        try:
            degree_constant = (2 * math.pi * 6371000)/360
            time.sleep(4)
            origin = self.onboardGPSData[-1]
            origin = [(origin[0] * degree_constant * math.cos((origin[1] * math.pi)/180)), (origin[1] * degree_constant)] # [longitude, latitude]
            last_time_entry = self.onboardGPSData[-1][2]
            while not self.terminate_flag.value: # Check terminate flag
                if(self.onboardGPSData[-1][2] != last_time_entry):
                    current_gps_data = self.onboardGPSData[-1]
                    current_imu_data = self.imuData[-1]

                    # Get current displacement and position value from imu and gps
                    gpsPos = [current_gps_data[0], current_gps_data[1]] # [longitude, latitude]
                    imuPos = [current_imu_data[0], current_imu_data[1]] # [x, y]

                    # Convert GPS to x, y coordinates
                    gpsPos = [(gpsPos[0] * degree_constant * math.cos((gpsPos[1] * math.pi)/180)) - origin[0], (gpsPos[1] * degree_constant) - origin[1]] # [x, y]

                    # Calculate GPS weight - some equation
                    satellite_weight = current_gps_data[3] * 0.5
                    if(current_gps_data[4] == 0):
                        current_gps_data = 200
                    hdop_weight = 2/(current_gps_data[4] / 100)
                    print('GPS Weight: ' + str(satellite_weight) + ' | ' + str(hdop_weight))
                    gps_weight = satellite_weight + hdop_weight

                    # Calculate IMU weight
                    imu_weight = 2

                    # Calculate weighted average of GPS and IMU calculated position
                    x = (gps_weight / (gps_weight + imu_weight)) * gpsPos[0] + (imu_weight / (gps_weight + imu_weight)) * imuPos[0]
                    y = (gps_weight / (gps_weight + imu_weight)) * gpsPos[1] + (imu_weight / (gps_weight + imu_weight)) * imuPos[1]

                    print('Odometry:' + str([x, y]))
                    self.odometryData.append([-x, y, current_imu_data[2], time.time(), origin, gps_weight, imu_weight])


                    # Update IMU calculated position
                    loc = [-x, y]
                    self.update_imuPos.append(loc)

                    # Update last_time_entry
                    last_time_entry = current_gps_data[2]
                    time.sleep(0.2)

        except Exception as e:
            print(time.time())
            print("Error in Odometry: " + str(e.with_traceback()))

        print("*  Odometry calculations terminated!")
    
    # TESTING
    def test(self): # For testing purposes
        print("*  TEST: Starting Odometry Calculations...")

        try:
            while not self.terminate_flag.value:
                print("Logging odometry data: " + str(self.get()))
                self.x += 1
                self.y += 2
                self.theta += 3
                self.timestamp += 5
                self.update(self.x, self.y, self.theta, self.timestamp)
                time.sleep(2)
        except Exception as e:
            print("Error in Odometry TEST: " + str(e))

        print("*  TEST: Odometry calculations terminated!")
    
if __name__ == "__main__":
    odometry = Odometry()
    odometry.main()