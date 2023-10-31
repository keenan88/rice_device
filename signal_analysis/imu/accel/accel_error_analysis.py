import matplotlib.pyplot as plt
import json, os
import numpy as np

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
    get_ipython().magic('reset -f')
except:
    pass


folder_path = '/home/keenan/Documents/rice_device/ros2_ws/src/accel_extractor/samples/'

most_recent_file = folder_path + max([name[:-5] for name in os.listdir(folder_path)]) + ".json"

imu_accel_data = []

with open(most_recent_file, 'r') as json_file:
    samples = json.load(json_file)


time_series = []
first_time = samples['seconds'][0] + samples['nanosecs'][0] / 1000 / 1000 / 1000
for second, nsec in zip(samples['seconds'], samples['nanosecs']):
    
    time_series.append(round(second + nsec / 1000 / 1000 / 1000 - first_time, 2))
    

plt.plot(time_series, samples['accelXs'])
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Acceleration reading X (m/s^2?)")
plt.title("IMU Acceleration over time")