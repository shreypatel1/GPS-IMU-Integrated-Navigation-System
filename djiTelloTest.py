import djitellopy
import time

tello = djitellopy.Tello()

tello.connect(wait_for_state=True)
#tello.takeoff()

#tello.move_left(100)
#tello.rotate_counter_clockwise(90)
#tello.move_forward(100)

#tello.land()
while True:
    print('Height: ' + str(tello.get_height()))
    print('Acceleration: ' + str(tello.get_acceleration_x()) + ' - ' + str(tello.get_acceleration_y()) + ' - ' + str(tello.get_acceleration_z()))
    print('R-P-Y: ' + str(tello.get_roll()) + ' - ' + str(tello.get_pitch()) + ' - ' + str(tello.get_yaw()))
    print('Speed: ' + str(tello.get_speed_x()) + ' - ' + str(tello.get_speed_y()) + ' - ' + str(tello.get_speed_z()))

    time.sleep(1)

# Roll, Pitch, Yaw works (degrees)
# Acceleration works (cm/s^2 : just why?)
# Speed probably only works when flying at high speed

tello.end()
