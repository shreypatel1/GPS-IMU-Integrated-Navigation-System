import time

class Odometry:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.terminate_flag = False

    def update(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def get(self):
        return self.x, self.y, self.theta
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_theta(self):
        return self.theta
    
    def main(self):
        print("Starting Odometry")
        while not self.terminate_flag: # Check terminate flag
            self.update(1, 2, 3)
            print(self.get())
            print(self.get_x())
            print(self.get_y())
            print(self.get_theta())

    def set_terminate_flag(self):
        self.terminate_flag = True
    
    def test(self):
        while True:
            print("Logging odometry data: " + str(self.get()))
            time.sleep(2)
    
if __name__ == "__main__":
    odometry = Odometry()
    odometry.main()