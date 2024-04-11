import time
import djitellopy
import threading
import keyboard
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

terminate_flag = False
current_yaw = 0.0
target_yaw = 90
output_values = []
error_values = []
integral_values = []
derivative_values = []
battery_percentage = []
yaw_values = {'current_yaw': [], 'target_yaw': []}
time_values = []

# MPC Parameters
N = 8  # Prediction horizon
dt = 0.1  # Time step

def termination_listener():
    global terminate_flag
    keyboard.wait('esc')
    print('Termination signal received! Terminating processes...')
    terminate_flag = True

def cost_function(u):
    # Cost function to minimize
    cost = 0
    for i in range(N):
        error = target_yaw - (current_yaw + u[i])
        cost += error ** 2
    return cost

def estimate_pose(tello):
    global current_yaw
    current_yaw = tello.get_yaw()
    print('Current Yaw:', current_yaw)

def plot_data():
    while True:
        plt.figure(figsize=(10, 8))

        # Plot for output, error, and integral
        plt.subplot(411)
        plt.plot(time_values, error_values, label='Error')
        plt.plot(time_values, integral_values, label='Integral')
        plt.plot(time_values, derivative_values, label='Derivative')
        plt.title('PID Components')
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.legend()

        # Plot the putput
        plt.subplot(412)
        plt.plot(time_values, output_values, label='Output')
        plt.title('Output')
        plt.xlabel('Time (s)')
        plt.ylabel('Value')

        # Plot for current yaw and target
        plt.subplot(413)
        plt.plot(time_values, yaw_values['current_yaw'], label='Current Yaw')
        plt.plot(time_values, yaw_values['target_yaw'], label='Target Yaw', linestyle='--')
        plt.axhline(y=90, color='gray', linestyle=':', linewidth=1)
        plt.title('Yaw Values')
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw')
        plt.legend()

        # Plot for battery level (bar graph)
        plt.subplot(414)
        plt.plot(time_values, battery_percentage)
        plt.title('Battery Percentage')
        plt.xlabel('Time (s)')
        plt.ylabel('Percentage')

        plt.tight_layout()
        plt.show()
        plt.pause(0.1)

def main():
    global terminate_flag, current_yaw

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    tello = djitellopy.Tello()
    tello.connect(wait_for_state=True)

    print("Starting Flight Navigation...")

    tello.takeoff()

    # Main loop
    try:
        data_time = 0
        previous_time = time.time()
        while not terminate_flag:
            # Calculate the time elapsed
            current_time = time.time()
            dt = current_time - previous_time
            previous_time = current_time
            data_time += dt

            # Estimate the pose of the drone
            print('------------------------------')
            estimate_pose(tello)

            # MPC optimization
            u0 = np.zeros(N)
            res = minimize(cost_function, u0, method='SLSQP')
            yaw_velocity = int(res.x[0] * 10)  # Use the first control input as the velocity

            # Append the data
            time_values.append(data_time)
            output_values.append(yaw_velocity)
            error_values.append(target_yaw - current_yaw)
            integral_values.append(np.sum(error_values) * dt)
            derivative_values.append((error_values[-1] - error_values[-2]) / dt if len(error_values) > 1 else 0)
            yaw_values['current_yaw'].append(current_yaw)
            yaw_values['target_yaw'].append(target_yaw)
            battery_percentage.append(tello.get_battery())

            # Send the velocity values to the drone
            tello.send_rc_control(0, 0, 0, yaw_velocity)

            # Sleep for a short period of time
            time.sleep(0.05)

    except Exception as e:
        print(e)
        print('Terminating Flight Navigation...')

    print("Terminating Flight Navigation...")

    tello.send_rc_control(0, 0, 0, 0)
    tello.land()
    tello.end()

if __name__ == "__main__":
    main()
    plot_data()
