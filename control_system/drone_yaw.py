import time
import djitellopy
import numpy as np
from scipy.optimize import minimize
import math

class YawControl:
    def __init__(self, terminate_flag, yawData, imuData, remoteGPSData, odometryData):
        self.terminate_flag = terminate_flag
        self.yawData = yawData
        self.imuData = imuData
        self.remoteGPSData = remoteGPSData
        self.odometryData = odometryData
        self.current_yaw = 0.0
        self.target_yaw = 0
        self.output_values = []
        self.yaw_values = {'current_yaw': [], 'target_yaw': []}
        self.time_values = []
        self.degree_constant = (2 * math.pi * 6371000)/360

        # MPC Parameters
        self.N = 8  # Prediction horizon
        self.dt = 0.09  # Time step

    def cost_function(self, u):
        # Cost function to minimize
        cost = 0
        for i in range(self.N):
            error = self.target_yaw - (self.current_yaw + u[i])
            cost += error ** 2
        return cost
    
    def estimate_pose(self):
        origin = self.odometryData[-1][4]
        target = self.remoteGPSData[-1]
        target = [(target[0] * self.degree_constant * math.cos((target[1] * math.pi)/180)) - origin[0], (target[1] * self.degree_constant) - origin[1]] # [x, y]
        current = self.odometryData[-1] # x, y, theta, timestamp, origin
        displacement = [target[0] - current[0], target[1] - current[1]]

        # Calculate the target yaw in degrees
        self.target_yaw = math.degrees(math.atan2(displacement[1], -displacement[0]))

        self.current_yaw = self.imuData[-1][2]

    def main(self):
        print("*  Starting Yaw Control...")

        # Main loop
        try:
            time.sleep(2)
            data_time = 0
            previous_time = time.time()
            last_yaw_time = 0
            while not self.terminate_flag.value:
                if(self.imuData[-1][3] != last_yaw_time):
                    # Calculate the time elapsed
                    current_time = time.time()
                    dt = current_time - previous_time
                    previous_time = current_time
                    data_time += dt

                    # Estimate the pose of the drone
                    #print('------------------------------')
                    self.estimate_pose()

                    # MPC optimization
                    u0 = np.zeros(self.N)
                    res = minimize(self.cost_function, u0, method='SLSQP')
                    yaw_velocity = int(res.x[0] * 10)  # Use the first control input as the velocity
                    if(yaw_velocity > 100):
                        yaw_velocity = 100
                    elif(yaw_velocity < -100):
                        yaw_velocity = -100

                    # Append the data
                    self.yawData.append([self.current_yaw, self.target_yaw, yaw_velocity, data_time])
                    #self.yaw_values['current_yaw'].append(self.current_yaw)
                    #self.yaw_values['target_yaw'].append(self.target_yaw)
                    #self.output_values.append(yaw_velocity)
                    #self.time_values.append(data_time)

                    # Update Yaw velocity values
                    #self.tello.value.send_rc_control(0, 0, 0, yaw_velocity)

                    # Sleep for a short period of time
                    time.sleep(0.05)
                else:
                    time.sleep(0.05)
        except Exception as e:
            print("Error in YAW Control: " + str(e.with_traceback()))
            print('*  Terminating Yaw Control...')

        print("*  Yaw Control Terminated!")

if __name__ == "__main__":
    gps_logger = YawControl()
    gps_logger.main()