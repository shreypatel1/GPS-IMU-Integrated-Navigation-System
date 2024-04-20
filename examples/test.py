import matplotlib.pyplot as plt
import numpy as np
import time
import pandas as pd
import math
from geopy.distance import distance
import os
import csv

imuN=1
remoteGPSData = [-84.521035, 33.937093]


# Load data
imuData = pd.read_csv('data_sets/csv_files5/imuData.csv', header=None).values.tolist()
onboardGPSData = pd.read_csv('data_sets/csv_files5/onboardGPSData.csv', header=None).values.tolist()
odometryData = pd.read_csv('data_sets/csv_files5/odometryData.csv', header=None).values.tolist()


degree_constant = (2 * math.pi * 6371000)/360
origin = onboardGPSData[0]
origin = [(origin[0] * degree_constant * math.cos((origin[1] * math.pi)/180)), (origin[1] * degree_constant)] # [longitude, latitude]
target_loc = [(remoteGPSData[0] * degree_constant * math.cos((remoteGPSData[1] * math.pi)/180)) - origin[0], (remoteGPSData[1] * degree_constant) - origin[1]] # [x, y]

# Define the directory to save CSV files
CSV_DIR = "data_sets/coordinate_csv_files"

# Create the directory if it does not exist
os.makedirs(CSV_DIR, exist_ok=True)

def save_list_as_csv(filename, data):
    with open(os.path.join(CSV_DIR, filename), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)


#------------------------------------------------------------------------------------------------
# Find coordinates for each data point (NOT WORKING FOR NOW)
def offset_coordinate(dx, dy):
    global onboardGPSData
    """
    Offset a latitude and longitude coordinate by dx meters to the east and dy meters to the north.
    
    Parameters:
        lat (float): Latitude of the original coordinate.
        lon (float): Longitude of the original coordinate.
        dx (float): Offset distance in meters to the east.
        dy (float): Offset distance in meters to the north.
    
    Returns:
        tuple: Latitude and longitude of the new coordinate after offset.
    """
    # Create a VincentyDistance object with the offset distances
    offset_dist = distance(meters=dx)
    
    # Calculate the new latitude and longitude using the offset distances
    new_latlon = offset_dist.destination((onboardGPSData[0][1], onboardGPSData[0][0]), bearing=90)  # East direction
    new_lat, new_lon = new_latlon.latitude, new_latlon.longitude
    
    offset_dist = distance(meters=dy)
    new_latlon = offset_dist.destination((new_lat, new_lon), bearing=0)  # North direction
    new_lat, new_lon = new_latlon.latitude, new_latlon.longitude
    
    return [new_lat, new_lon]

imuCoord = []
for data in imuData:
    imuCoord.append(offset_coordinate(-data[0], -data[1]))

odomCoord = []
for data in odometryData:
    odomCoord.append(offset_coordinate(data[0], -data[1]))

# Save the offset coordinates as CSV files
#save_list_as_csv("imu.csv", imuCoord)
#save_list_as_csv("odometry.csv", odomCoord)
#save_list_as_csv("onboardGPS.csv", ([data[1], data[0]] for data in onboardGPSData))
#------------------------------------------------------------------------------------------------

weight_data = []
previous_time = odometryData[1][3]
current_time = 0
for data in odometryData:
    if (data[3] != previous_time) and (data[3] != 0):
        current_time += data[3] - previous_time
        weight_data.append([current_time, (data[5] / (data[5] + data[6])), (data[6] / (data[5] + data[6]))])
        previous_time = data[3]
#print(weight_data[-1])
#print(current_time)


#fig, axs = plt.subplots(3, 2, figsize=(8, 10))

#axs = axs.flatten()

plt.figure(figsize=(10, 8))

# Location (x-y) graph
plt.subplot(211)
plt.plot([-data[0] for data in imuData], [-data[1] for data in imuData], label='IMU Data')
plt.plot([((data[0] * degree_constant * math.cos((data[1] * math.pi)/180)) - origin[0]) for data in onboardGPSData], [-((data[1] * degree_constant) - origin[1]) for data in onboardGPSData], label='Onboard GPS Data')
plt.plot([data[0] for data in odometryData], [-data[1] for data in odometryData], label='Odometry Data')
plt.plot(target_loc[0], target_loc[1], 'ro', label='Target')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Drone Position (X-Y)')
plt.legend()
plt.grid(True)

# GPS weight graph
#plt.subplot(321)
#axs[2].plot([data[0] for data in weight_data], [data[1] for data in weight_data], label='GPS Weight')
#plt.plot([data[0] for data in weight_data], [data[2] for data in weight_data], label='IMU Weight')
##axs[2].set_xlabel('Time (s)')
#axs[2].set_ylabel('Weight')
#axs[2].set_title('GPS Weightage')
#axs[2].legend()
#axs[2].grid(True)


# IMU weight graph
#plt.subplot(322)
#plt.plot([data[0] for data in weight_data], [data[1] for data in weight_data], label='GPS Weight')
#axs[3].plot([data[0] for data in weight_data], [data[2] for data in weight_data], label='IMU Weight', color='orange')
#axs[3].set_xlabel('Time (s)')
#axs[3].set_ylabel('Weight')
#axs[3].set_title('IMU Weightage')
#axs[3].legend()
#axs[3].grid(True)


plt.tight_layout()
plt.show()