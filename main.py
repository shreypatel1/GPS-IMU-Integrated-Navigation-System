import csv
import os
import multiprocessing
import threading
import keyboard
import time
import djitellopy
import math
import numpy as np
import serial
from scipy.optimize import minimize
from data_logging.onboard_gps_logger import OnboardGPSLogger
#from data_logging.remote_gps_reciever import RemoteGPSReciever
from data_logging.odometry import Odometry
from control_system.drone_yaw import YawControl
import matplotlib.pyplot as plt

# Define the directory to save CSV files
CSV_DIR = "last_flight_data"

# Create the directory if it does not exist
os.makedirs(CSV_DIR, exist_ok=True)

# Function to save list as CSV
def save_list_as_csv(filename, data):
    with open(os.path.join(CSV_DIR, filename), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)


position = [0.0, 0.0]
yaw = 0
velocity = [0.0, 0.0]
acceleration_bias = [0.0, 0.0]
bias_alpha = 0.5


    
def estimate_pose(dt, tello, imuData):
    global position, yaw, velocity, acceleration_bias, bias_alpha

    # Create a variable to store the acceleration readings
    acceleration = [0.0, 0.0]

    yaw = tello.get_yaw()

    # Update the acceleration readings
    acceleration_x = tello.get_acceleration_x() / 10
    acceleration_y = tello.get_acceleration_y() / 10

    acceleration[0] = acceleration_x * math.cos(math.radians(yaw)) - acceleration_y * math.sin(math.radians(yaw))
    acceleration[1] = acceleration_x * math.sin(math.radians(yaw)) + acceleration_y * math.cos(math.radians(yaw))

    # Apply a bias filter to the acceleration readings
    acceleration_bias[0] = bias_alpha * acceleration_bias[0] + (1 - bias_alpha) * acceleration[0]
    acceleration_bias[1] = bias_alpha * acceleration_bias[1] + (1 - bias_alpha) * acceleration[1]
    acceleration[0] -= acceleration_bias[0]
    acceleration[1] -= acceleration_bias[1]

    # Update the velocity
    velocity[0] += -acceleration[0] * dt
    velocity[1] += acceleration[1] * dt
    #velocity[0] += math.trunc(-acceleration[0] * dt)
    #velocity[1] += math.trunc(acceleration[1] * dt)

    velocity[0] = math.trunc(velocity[0] * 100) / 100
    velocity[1] = math.trunc(velocity[1] * 100) / 100

    # Update the position
    position[0] += velocity[0] * dt
    position[1] += velocity[1] * dt
        
        
    timestamp = time.time()

    imuData.append([position[0], position[1], yaw, timestamp])

    #print('Position: ' + str(current_x) + ' | ' + str(current_y))
    #print('Acceleration: ' + str(acceleration[0]) + ' | ' + str(acceleration[1]))
    #print('Velocity: ' + str(current_velocity))
    #print('Dt: ' + str(dt))
    
def imu_main(terminate_flag, tello, update_imuPos, imuData):

    print("*  Starting IMU Logger...")

    try:
        previous_time = time.time()
        last_update = [0.0, 0.0]
        while not terminate_flag.value: # Check terminate flag
            # Calculate the time elapsed
            current_time = time.time()
            dt = current_time - previous_time
            #print('IMU dt: ' + str(dt))
            previous_time = current_time

            # Estimate the pose of the drone
            if(update_imuPos[-1] != last_update):
                last_update = update_imuPos[-1]
                position = update_imuPos[-1]
                imuData.append([position[0], position[1], tello.get_yaw(), time.time()])
            else:
                estimate_pose(dt, tello, imuData)

            #print([position[0], position[1], yaw])


            time.sleep(0.05)
    except Exception as e:
        print("Error in IMU Logger: " + str(e.with_traceback()))
        print("Terminating IMU Logger...")

    terminate_flag.value = True

    print("*  IMU Logger terminated!")


# Multiprocess variables
terminate_flag = None
onboardGPSData = None
remoteGPSData = None
odometryData = None

# MPC Parameters
N = 10  # Prediction horizon
dt = 1  # Time step
remoteGPSData = [
    [-84.521035, 33.937093],
]
current = 0.0
target = 0.0
degree_constant = (2 * math.pi * 6371000)/360
meter_constant = 360/(2 * math.pi * 6371000)
time_values = []
output_values = []
error_values = []
integral_values = []
derivative_values = []


# Handle Ctrl+C
def termination_listener():
    global terminate_flag
    keyboard.wait('esc')
    print('Termination signal received! Terminating processes...')
    terminate_flag.value = True


def cost_function(u):
    global current, target, N

    # Cost function to minimize
    cost = 0
    for i in range(N):
        error = target - (current + u[i])
        cost += error ** 2
    return cost


def estimate_dist(odomData):
    global remoteGPSData, current, target, degree_constant
    origin = odomData[4]
    target_loc = [(remoteGPSData[-1][0] * degree_constant * math.cos((remoteGPSData[-1][1] * math.pi)/180)) - origin[0], (remoteGPSData[-1][1] * degree_constant) - origin[1]] # [x, y]
    displacement = [target_loc[0] - odomData[0], target_loc[1] - odomData[1]]
    # Do pythagorean theorem to get distance
    current = -(math.sqrt(displacement[0] ** 2 + displacement[1] ** 2))


def plot_data(imuData, onboardGPSData, odometryData, yawData):
    global time_values, output_values, error_values, integral_values, derivative_values, degree_constant, meter_constant

    origin = odometryData[-1][4]
    target_loc = [(remoteGPSData[-1][0] * degree_constant * math.cos((remoteGPSData[-1][1] * math.pi)/180)) - origin[0], (remoteGPSData[-1][1] * degree_constant) - origin[1]] # [x, y]

    # Convert x and y of origin, imuData, odometry to lat and lon
    imuData = [[(data[0] + origin[0]) * meter_constant * (1/math.cos(((data[1] + origin[1]) * meter_constant * math.pi)/180)), (data[1] + origin[1]) * meter_constant, data[2], data[3]] for data in imuData]
    odometryData = [[(data[0] + origin[0]) * meter_constant * (1/math.cos(((data[1] + origin[1]) * meter_constant * math.pi)/180)), (data[1] + origin[1]) * meter_constant, data[2], data[3]] for data in odometryData]



    plt.figure(figsize=(10, 8))

    # Location (x-y) graph
    plt.subplot(311)
    plt.plot([data[0] for data in imuData], [data[1] for data in imuData], label='IMU Data')
    plt.plot([((data[0] * degree_constant * math.cos((data[1] * math.pi)/180)) - origin[0]) for data in onboardGPSData], [((data[1] * degree_constant) - origin[1]) for data in onboardGPSData], label='Onboard GPS Data')
    plt.plot([data[0] for data in odometryData], [data[1] for data in odometryData], label='Odometry Data')
    plt.plot(target_loc[0], target_loc[1], 'ro', label='Target')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Drone Position (X-Y)')
    plt.legend()
    plt.grid(True)

    # PID components vs. time
    plt.subplot(312)
    plt.plot(time_values, error_values, label='Error')
    plt.plot(time_values, integral_values, label='Integral')
    plt.plot(time_values, derivative_values, label='Derivative')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('PID Components vs. Time')
    plt.legend()

    # Output vs. time (forward backward velocity output)
    plt.subplot(313)
    plt.plot(time_values, output_values, label='Forward Backward Velocity Output')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (cm/s)')  # Assuming the unit of velocity is cm/s
    plt.title('Forward Velocity vs. Time (Forward Backward Velocity)')
    plt.legend()

    plt.tight_layout()


    plt.figure(figsize=(10, 8))

    # Yaw values vs. time
    plt.subplot(211)
    plt.plot([data[3] for data in yawData], [data[0] for data in yawData], label='Current Yaw')
    plt.plot([data[3] for data in yawData], [data[1] for data in yawData], label='Target Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (degrees)')
    plt.title('Yaw Values vs. Time')
    plt.legend()

    # Yaw output vs. time
    plt.subplot(212)
    plt.plot([data[3] for data in yawData], [data[2] for data in yawData], label='Yaw Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Velocity (degrees/s)')
    plt.title('Yaw Velocity vs. Time')
    plt.legend()

    plt.tight_layout()  # Adjust layout to prevent overlapping
    plt.show()



def main():
    global terminate_flag, remoteGPSData, current, target, time_values, output_values, error_values, integral_values, derivative_values

    # Create a manager for shared variables
    manager = multiprocessing.Manager()
    terminate_flag = manager.Value('b', False)
    update_imuPos = manager.list([[0.0, 0.0]])
    imuData = manager.list([[0.0, 0.0, 0, 0]]) # [x, y, yaw, timestamp]
    onboardGPSData = manager.list([]) # [longitude, latitude, timestamp, satellites, hdop, speed]
    #remoteGPSData = manager.list([[-84.5214510, 33.9370979, 0, 12]])
    odometryData = manager.list([[0.0, 0.0, 0, 0, [0.0, 0.0], 0.0, 0.0]]) # [x, y, theta, timestamp, origin(x, y), gps_weight, imu_weight]
    yawData = manager.list([[0, 0, 0, 0]]) # [current_yaw, target_yaw, yaw_velocity, timestamp]

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    # Tello drone object
    tello = djitellopy.Tello()
    tello.connect(wait_for_state=True)


    print("Starting Flight Navigation...")


    # Instantiate the classes
    onboard_gps_logger = OnboardGPSLogger(terminate_flag, onboardGPSData, '/dev/tty.usbmodem113301', 9600) # This logs the onboard GPS data
    #remote_gps_reciever = RemoteGPSReciever(terminate_flag, remoteGPSData, '10.101.180.10', 4050) # This recieves the remote GPS data
    odometry = Odometry(terminate_flag, update_imuPos, odometryData, imuData, onboardGPSData) # This calculates the drone odometry
    yawControl = YawControl(terminate_flag, yawData, imuData, remoteGPSData, odometryData) # This controls the yaw of the drone


    # Start the subprocesses
    #imu_process = multiprocessing.Process(target=imu_logger.main)
    #imu_process.start()
    imu_process = threading.Thread(target=imu_main, args=(terminate_flag, tello, update_imuPos, imuData))
    imu_process.start()

    oGPS_process = multiprocessing.Process(target=onboard_gps_logger.main)
    oGPS_process.start()

    #rGPS_process = multiprocessing.Process(target=remote_gps_reciever.main)
    #rGPS_process.start()
    
    odometry_process = multiprocessing.Process(target=odometry.main)
    odometry_process.start()

    yaw_process = multiprocessing.Process(target=yawControl.main)
    yaw_process.start()


    tello.takeoff()


    # Loop through the waypoints until the terminate flag is set
    try:
        time.sleep(5)
        data_time = 0
        previous_time = time.time()
        while not terminate_flag.value:
            # Calculate the time elapsed
            current_time = time.time()
            dt = current_time - previous_time
            previous_time = current_time
            data_time += dt

            # Estimate the pose of the drone
            print('------------------------------')
            estimate_dist(odometryData[-1])

            # MPC optimization
            u0 = np.zeros(N)
            res = minimize(cost_function, u0, method='SLSQP')
            forward_backward_velocity = int(res.x[0] * 10)  # Use the first control input as the velocity
            if(forward_backward_velocity > 100):
                forward_backward_velocity = 100
            elif(forward_backward_velocity < -100):
                forward_backward_velocity = -100

            # Append the data
            time_values.append(data_time)
            output_values.append(forward_backward_velocity)
            error_values.append(target - current)
            integral_values.append(np.sum(error_values) * dt)
            derivative_values.append((error_values[-1] - error_values[-2]) / dt if len(error_values) > 1 else 0)
            print('Battery: ' + str(tello.get_battery()))

            # Send the velocity values to the drone
            tello.send_rc_control(0, forward_backward_velocity, 0, yawData[-1][2])

            # Sleep for a short period of time
            time.sleep(0.05)
    except Exception as e:
        print(e)
        print('Terminating Flight Navigation...')
    

    print("Terminating Flight Navigation...")

    tello.send_rc_control(0, 0, 0, 0)

    tello.land()
    

    # Wait for the processes to terminate
    oGPS_process.join()
    #rGPS_process.join()
    odometry_process.join()
    imu_process.join()
    yaw_process.join()

    tello.end()

    print("Flight Navigation terminated!")

    # Plot the data
    save_list_as_csv('imuData.csv', imuData)
    save_list_as_csv('onboardGPSData.csv', onboardGPSData)
    save_list_as_csv('odometryData.csv', odometryData)
    save_list_as_csv('yawData.csv', yawData)
    save_list_as_csv('updateIMU.csv', update_imuPos)
    save_list_as_csv('time_values.csv', [[val] for val in time_values])
    save_list_as_csv('output_values.csv', [[val] for val in output_values])
    save_list_as_csv('error_values.csv', [[val] for val in error_values])
    save_list_as_csv('integral_values.csv', [[val] for val in integral_values])
    save_list_as_csv('derivative_values.csv', [[val] for val in derivative_values])
    plot_data(imuData, onboardGPSData, odometryData, yawData)


if __name__ == '__main__':
    main()