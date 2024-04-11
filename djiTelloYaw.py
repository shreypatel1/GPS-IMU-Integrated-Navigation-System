import time
import djitellopy
import threading
import keyboard
import matplotlib.pyplot as plt
import pynput

terminate_flag = False
yawData = [0, 0]
current_yaw = 0.0
current_velocity = 0

output_values = []
error_values = []
integral_values = []
derivative_values = []
battery_percentage = []
yaw_values = {'current_yaw': [], 'target_yaw': []}
time_values = []



def termination_listener():
    global terminate_flag
    keyboard.wait('esc')
    print('Termination signal received! Terminating processes...')
    terminate_flag = True

def pid_controller(target, current, prev_error, integral, dt, kp, ki, kd):

    error = target - current # Calculate the error
    integral += error * dt # Accumulate the integral error
    derivative = 0.0
    if(dt > 0.05):
        derivative = (error - prev_error) / dt # Calculate the derivative error
    output = kp * error + ki * integral + kd * derivative # Calculate the output

    return output, error, integral, derivative


def estimate_pose(tello, dt, target_yaw):
    global terminate_flag
    global current_yaw, current_velocity

    # Update the position
    current_yaw = tello.get_yaw()

    yaw_values['current_yaw'].append(current_yaw)
    yaw_values['target_yaw'].append(target_yaw)

    #print('Position: ' + str(current_yaw))
    #print('Target: ' + str(target_yaw))
    print('Dt: ' + str(dt))


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
    global terminate_flag
    global current_yaw

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    tello = djitellopy.Tello()
    tello.connect(wait_for_state=True)

    print("Starting Flight Navigation...")


    # Define PID parameters
    kp_yaw, ki_yaw, kd_yaw = 0.35, 0.15, 0.18
    prev_error_yaw, integral_yaw = 0.0, 0.0
    target_yaw = 90


    print(tello.get_battery())
    tello.takeoff()


    # Main loop
    try:
        data_time = 0
        previous_time = time.time()
        while not terminate_flag:
            # Calculate the time elapsed
            current_time = time.time()
            dt = current_time - previous_time
            #print('dt: ' + str(dt))
            previous_time = current_time
            data_time += dt

            time_values.append(data_time)

            # Estimate the pose of the drone
            print('------------------------------')
            estimate_pose(tello, dt, target_yaw)

            # Calculate the PID output
            output_yaw, prev_error_yaw, integral_yaw, derivative_yaw = pid_controller(target_yaw, current_yaw, prev_error_yaw, integral_yaw, dt, kp_yaw, ki_yaw, kd_yaw)

            # Append output, error, and integral values to the lists
            error_values.append(prev_error_yaw)
            integral_values.append(integral_yaw)
            derivative_values.append(derivative_yaw)

            # Append the battery percentage to the list
            battery_percentage.append(tello.get_battery())

            # Convert PID outputs to velocity values for each axis
            if(output_yaw > 10): 
                output_yaw = 10
            elif(output_yaw < -10):
                output_yaw = -10

            yaw_velocity = int(output_yaw * 10)
            output_values.append(int(output_yaw * 10))
            
            print('Output: ' + str(output_yaw * 10))
            print('Prev Error: ' + str(prev_error_yaw))
            print('Integral: ' + str(integral_yaw))
            print('------------------------------')


            # Send the velocity values to the drone
            tello.send_rc_control(0, 0, 0, yaw_velocity)

            # Sleep for a short period of time
            time.sleep(0.05)         

            # Break the loop if yaw has been consistent for a while
            if len(yawData) > 10:
                if all(yaw == target_yaw for yaw in yawData):
                    print('Yaw has been consistent for a while.')
                    break
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