# GPS-IMU-Integrated-Navigation-System

This project is a GPS-IMU integrated navigation system for the DJI Tello drone. The drone is controlled using the DJITelloPy library, and the GPS and IMU data are collected using the BN-180 GPS module and the in-built Tello drone IMU module, respectively.

### Links:
* *DJITelloPy*: https://djitellopy.readthedocs.io/en/latest/
* *GPSVisualizer*: https://www.gpsvisualizer.com/map_input



## Running the project:
If running this project on a Mac, you will need to run `main.py` as an administrator. This is because the termination button uses the `keyboard` module, which requires admin privileges to run.
```bash
sudo python main.py
```

> DO NOT RUN any of the other files in the project directory. They are not meant to be run individually, and will not work as expected.


## Terminating the project:

Use the `esc` key when terminating the program.
This will ensure that the program is terminated properly and all the processes finish.

> DO NOT USE `ctrl + c` to terminate the program.


## Clearing cached files:
Use this command for clearing all the compiled python files (`.pyc`) in the project directory:
```bash
find . -name "*.pyc" -exec rm -f {} \;
```

If you are having difficulties clearing the cached files because of permission issues, use the following command:
```bash
find . -name "*.pyc" -exec sudo rm -f {} \;
```
