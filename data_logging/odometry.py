import time

class Odometry:
    def __init__(self, terminate_flag, odometryData):
        self.terminate_flag = terminate_flag
        self.x = 0.0
        self.y = 0.0
        self.theta = 0
        self.timestamp = 0
        self.odometryData = odometryData

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
            while not self.terminate_flag.value: # Check terminate flag
                self.update(1, 2, 3)
                print(self.get())
                time.sleep(2)
        except Exception as e:
            print("Error in Odometry: " + str(e))

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