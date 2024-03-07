import time
import tellopy
import csv

file = None
write_header = True

def handler(event, sender, data, **args):
    # Open the CSV file in append mode
    print(data)
    with open('tello_data.csv', 'a') as file:
        writer = csv.writer(file)
        
        writer.writerow([time.time(), data])

def test():
    drone = tellopy.Tello()
    try:
        drone.subscribe(drone.EVENT_LOG_RAWDATA, handler)
        drone.record_log_data()

        drone.connect()
        drone.wait_for_connection(60.0)
        #drone.takeoff()
        time.sleep(5)
        #drone.clockwise(100)
        time.sleep(5)
        #drone.clockwise(0)
        #drone.down(50)
        time.sleep(2)
        #drone.up(50)
        time.sleep(2)
        #drone.up(0)
        #drone.land()
        time.sleep(5)
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()

if __name__ == '__main__':
    test()