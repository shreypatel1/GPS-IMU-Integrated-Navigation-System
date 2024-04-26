import threading
import keyboard
import djitellopy
import time
import math


# Multiprocess variables
terminate_flag = False
imuData = None


# Handle Ctrl+C
def termination_listener():
    global terminate_flag
    keyboard.wait('esc')
    print('Termination signal received! Terminating processes...')
    terminate_flag = True


def main():
    global terminate_flag

    # Start the termination listener
    termination_thread = threading.Thread(target=termination_listener)
    termination_thread.start()

    # Connect to the Tello drone
    tello = djitellopy.Tello()
    tello.connect(wait_for_state=True)

    print("Starting Flight Navigation...")

    #tello.takeoff()
    x_velocity, y_velocity = 0.0, 0.0
    x_position, y_position = 0.0, 0.0

    acceleration_bias_x, acceleration_bias_y = 0.0, 0.0
    bias_alpha = 0.05

    # Wait for the termination signal
    previous_time = time.time()
    while not terminate_flag: # Check terminate flag
        current_time = time.time()
        dt = current_time - previous_time
        previous_time = current_time

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

        #x_velocity += math.trunc(-acceleration_x * dt)
        #y_velocity += math.trunc(acceleration_y * dt)
        x_velocity += -acceleration_x * dt
        y_velocity += acceleration_y * dt

        x_velocity = math.trunc(x_velocity * 100) / 100
        y_velocity = math.trunc(y_velocity * 100) / 100

        x_position += x_velocity * dt
        y_position += y_velocity * dt
            
        #print('Height: ' + str(tello.get_height()))
        #print('State: ' + str(tello.get_current_state()))
        print("Battery: " + str(tello.get_battery()) + "%")
        print('Acceleration: ' + str(acceleration_x) + ' | ' + str(acceleration_y))
        #print('Filtered Acceleration: ' + str(filtered_acceleration_x) + ' | ' + str(filtered_acceleration_y))
        print('Velocity: ' + str(x_velocity) + ' | ' + str(y_velocity))
        print('Position: ' + str(x_position) + ' | ' + str(y_position))
        print('R-P-Y: ' + str(tello.get_roll()) + ' | ' + str(tello.get_pitch()) + ' | ' + str(tello.get_yaw()))
        #print('Speed: ' + str(tello.get_speed_x()) + ' | ' + str(tello.get_speed_y()) + ' | ' + str(tello.get_speed_z()))

        time.sleep(0.2)

    print("Terminating Flight Navigation...")

    #tello.land()

    tello.end()

    print("Flight Navigation terminated!")


if __name__ == '__main__':
    main()