import time
import djitellopy
import threading
import keyboard
import math

terminate_flag = False

current_x, current_y = 0.0, 0.0
initial_velocity = [0.0, 0.0]
current_velocity = initial_velocity.copy()
acceleration_bias_x, acceleration_bias_y = 0.0, 0.0
bias_alpha = 0.5

def termination_listener():
    global terminate_flag
    keyboard.wait('esc')
    print('Termination signal received! Terminating processes...')
    terminate_flag = True

def pid_controller(target, current, prev_error, integral, dt, kp, ki, kd):

    error = target - current # Calculate the error
    integral += error * dt # Accumulate the integral error
    derivative = 0.0
    if(dt != 0.0):
        derivative = (error - prev_error) / dt # Calculate the derivative error
    output = kp * error + ki * integral + kd * derivative # Calculate the output

    return output, error, integral


def log_imu(tello):
    global terminate_flag

    while not terminate_flag:
        print('Height: ' + str(tello.get_height()))
        print('Acceleration: ' + str(tello.get_acceleration_x()) + ' - ' + str(tello.get_acceleration_y()) + ' - ' + str(tello.get_acceleration_z()))
        print('R-P-Y: ' + str(tello.get_roll()) + ' - ' + str(tello.get_pitch()) + ' - ' + str(tello.get_yaw()))
        print('Speed: ' + str(tello.get_speed_x()) + ' - ' + str(tello.get_speed_y()) + ' - ' + str(tello.get_speed_z()))

        time.sleep(0.1)

    # Roll, Pitch, Yaw works (degrees)
    # Acceleration works (cm/s^2 : just why?)
    # Speed only works when flying at high speed (not sensitive enough)


def estimate_pose(tello, dt):
    global terminate_flag
    global current_x, current_y, current_velocity, acceleration_bias_x, acceleration_bias_y, bias_alpha

    yaw = tello.get_yaw()

    acceleration_xi = tello.get_acceleration_x() / 10
    acceleration_yi = tello.get_acceleration_y() / 10

    acceleration_x = acceleration_xi * math.cos(math.radians(yaw)) - acceleration_yi * math.sin(math.radians(yaw))
    acceleration_y = acceleration_xi * math.sin(math.radians(yaw)) + acceleration_yi * math.cos(math.radians(yaw))

    print('Original Acceleration: ' + str(acceleration_x) + ' | ' + str(acceleration_y))

    acceleration_bias_x = bias_alpha * acceleration_bias_x + (1 - bias_alpha) * acceleration_x
    acceleration_bias_y = bias_alpha * acceleration_bias_y + (1 - bias_alpha) * acceleration_y
    acceleration_x -= acceleration_bias_x
    acceleration_y -= acceleration_bias_y
    
    #print('Second Acceleration: ' + str(acceleration_x) + ' | ' + str(acceleration_y))

    #acceleration_x = math.trunc(acceleration_x)
    #acceleration_y = math.trunc(acceleration_y)

    #current_velocity[0] += math.trunc(-acceleration_x * dt)
    #current_velocity[1] += math.trunc(acceleration_y * dt)
    current_velocity[0] += -acceleration_x * dt
    current_velocity[1] += acceleration_y * dt

    current_velocity[0] = math.trunc(current_velocity[0] * 100) / 100
    current_velocity[1] = math.trunc(current_velocity[1] * 100) / 100

    current_x += current_velocity[0] * dt
    current_y += current_velocity[1] * dt

    print('Position: ' + str(current_x) + ' | ' + str(current_y))
    #print('Acceleration: ' + str(acceleration[0]) + ' | ' + str(acceleration[1]))
    print('Velocity: ' + str(current_velocity))
    print('R-P-Y: ' + str(tello.get_roll()) + ' | ' + str(tello.get_pitch()) + ' | ' + str(tello.get_yaw()))
    #print('Dt: ' + str(dt))



def main():
    global terminate_flag
    global current_x, current_y

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    tello = djitellopy.Tello()
    tello.connect(wait_for_state=True)

    print("Starting Flight Navigation...")


    kp_x, ki_x, kd_x = 1.0, 0.5, 0.0
    kp_y, ki_y, kd_y = 1.0, 0.5, 0.0

    prev_error_x, integral_x = 0.0, 0.0
    prev_error_y, integral_y = 0.0, 0.0

    target_x, target_y = 0.0, -1.0


    tello.takeoff()


    # Main loop
    try:
        previous_time = time.time()
        count = 0
        while not terminate_flag:
            # Calculate the time elapsed
            current_time = time.time()
            dt = current_time - previous_time
            #print('dt: ' + str(dt))
            previous_time = current_time

            # Estimate the pose of the drone
            estimate_pose(tello, dt)

            # Calculate the PID output
            output_x, prev_error_x, integral_x = pid_controller(target_x, current_x, prev_error_x, integral_x, dt, kp_x, ki_x, kd_x)
            output_y, prev_error_y, integral_y = pid_controller(target_y, current_y, prev_error_y, integral_y, dt, kp_y, ki_y, kd_y)

            # Convert PID outputs to velocity values for each axis
            left_right_velocity = int(output_x * 100)
            forward_backward_velocity = int(output_y * 100)
            print('Output: ' + str(left_right_velocity) + ' | ' + str(forward_backward_velocity))

            # Send the velocity values to the drone
            tello.send_rc_control(-forward_backward_velocity, left_right_velocity, 0, 0)
            #if(count < 10):
                #tello.send_rc_control(100, 0, 0, 0)
            #else:
                #tello.send_rc_control(0, 0, 0, 0)

            # Sleep for a short period of time
            count += 1
            time.sleep(0.1)         
    except Exception as e:
        print(e)
        print('Terminating Flight Navigation...')


    print("Terminating Flight Navigation...")

    tello.send_rc_control(0, 0, 0, 0)

    tello.land()

    tello.end()


if __name__ == "__main__":
    main()