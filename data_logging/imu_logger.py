import time
import math

class IMU_Logger:
    def __init__(self, terminate_flag, tello, update_imuPos, imuData):
        self.terminate_flag = terminate_flag
        self.tello = tello
        self.update_imuPos = update_imuPos
        self.position = [0.0, 0.0]
        #self.roll = 0
        #self.pitch = 0
        self.yaw = 0
        #self.timestamp = 0
        self.velocity = [0.0, 0.0]
        self.acceleration_bias = [0.0, 0.0]
        self.bias_alpha = 0.5
        self.imuData = imuData

    def update(self, position, roll, pitch, yaw, timestamp):
        self.position = position
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.timestamp = timestamp
        self.imuData.append([self.position[0], self.position[1], roll, pitch, yaw, timestamp])

    def get(self):
        return self.x, self.y, self.yaw, self.timestamp
    
    def estimate_pose(self, dt, tello):

        # Create a variable to store the acceleration readings
        acceleration = [0.0, 0.0]

        yaw = tello.get_yaw()

        # Update the acceleration readings
        acceleration_x = tello.get_acceleration_x()
        acceleration_y = tello.get_acceleration_y()

        acceleration[0] = acceleration_x * math.cos(math.radians(yaw)) - acceleration_y * math.sin(math.radians(yaw))
        acceleration[1] = acceleration_x * math.sin(math.radians(yaw)) + acceleration_y * math.cos(math.radians(yaw))

        # Apply a bias filter to the acceleration readings
        self.acceleration_bias[0] = self.bias_alpha * self.acceleration_bias[0] + (1 - self.bias_alpha) * acceleration[0]
        self.acceleration_bias[1] = self.bias_alpha * self.acceleration_bias[1] + (1 - self.bias_alpha) * acceleration[1]
        acceleration[0] -= self.acceleration_bias[0]
        acceleration[1] -= self.acceleration_bias[1]

        # Update the velocity
        #self.velocity[0] += -acceleration[0] * dt
        #self.velocity[1] += -acceleration[1] * dt
        self.velocity[0] += math.trunc(-acceleration[0] * dt)
        self.velocity[1] += math.trunc(acceleration[1] * dt)

        # Update the position
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        

        #roll = self.tello.get_roll()
        #pitch = self.tello.get_pitch()
        
        timestamp = time.time()

        self.imuData.append([self.position[0], self.position[1], yaw, timestamp])

        #print('Position: ' + str(current_x) + ' | ' + str(current_y))
        #print('Acceleration: ' + str(acceleration[0]) + ' | ' + str(acceleration[1]))
        #print('Velocity: ' + str(current_velocity))
        #print('Dt: ' + str(dt))
    
    def main(self):
        print("*  Starting IMU Logger...")

        try:
            previous_time = time.time()
            last_update = [0.0, 0.0]
            while not self.terminate_flag.value: # Check terminate flag
                # Calculate the time elapsed
                current_time = time.time()
                dt = current_time - previous_time
                print('IMU dt: ' + str(dt))
                previous_time = current_time

                # Estimate the pose of the drone
                if(self.update_imuPos != last_update):
                    last_update = self.update_imuPos
                    self.position = self.update_imuPos
                    self.imuData.append([self.position[0], self.position[1], self.tello.get_yaw(), time.time()])
                else:
                    self.estimate_pose(dt, self.tello)
                
                print([self.position[0], self.position[1], self.yaw])

                time.sleep(0.05)
        except Exception as e:
            print("Error in IMU Logger: " + str(e.with_traceback()))
            print("Terminating IMU Logger...")

        self.terminate_flag.value = True

        print("*  IMU Logger terminated!")
    
if __name__ == "__main__":
    imu_logger = IMU_Logger()
    imu_logger.main()