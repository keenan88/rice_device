import matplotlib.pyplot as plt
import json, os
import numpy as np

try:
    from IPython import get_ipython
    #get_ipython().magic('clear')
    get_ipython().magic('reset -f')
except:
    pass


    
def plot_vel(time, accel):
    plt.plot(time, accel)
    plt.grid()
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity reading X (m/s)")
    plt.title("Extrapolated velocity over time")
    #plt.show()
    
def plot_pos(time, accel):
    plt.plot(time, accel)
    plt.grid()
    plt.xlabel("Time (s)")
    plt.ylabel("Extrapolated Pos (m)")
    plt.title("Extrapolated position over time")
    #plt.show()

def get_RMSE(signal):
    return round(np.sqrt(np.mean(np.square(signal))), 5)

folder_path = '/home/keenan/Documents/rice_device/ros2_ws/src/accel_extractor/samples/'

most_recent_file = folder_path + max([name[:-5] for name in os.listdir(folder_path)]) + ".json"

imu_accel_data = []

with open(most_recent_file, 'r') as json_file:
    samples = json.load(json_file)


time_series = []
first_time = samples['seconds'][0] + samples['nanosecs'][0] / 1000 / 1000 / 1000
for second, nsec in zip(samples['seconds'], samples['nanosecs']):
    
    time_series.append(round(second + nsec / 1000 / 1000 / 1000 - first_time, 2))


fig_unfiltered, ax_unfiltered = plt.subplots()
fig_filtered, ax_filtered = plt.subplots()

window_size = 10
for key in ['accelXs', 'accelYs', 'accelZs']:
    accs = samples[key]
    print(key)
    
    
    filtered_accs = []
    
    for i in range(window_size, len(accs)):
        
        filtered_acc = sum(accs[i - window_size : i]) / window_size
        filtered_accs.append(filtered_acc)
    
    times_for_filtered = time_series[window_size:]
    
    """
    accelerations = accs
    vels = [0] # velocities at each time step
    
    for i in range(1, len(accelerations)):
        accel_over_step = (accelerations[i] + accelerations[i - 1])/2
        time_step = time_series[i] - time_series[i - 1]
        delta_vel = accel_over_step * time_step
        
        vels.append(delta_vel + vels[i - 1]) 
        
    positions = [0]
    
    for i in range(1, len(vels)):
        
        velocity_over_step = (vels[i] + vels[i - 1])/2
        time_step = time_series[i] - time_series[i - 1]
        delta_pos = velocity_over_step * time_step
        new_pos = delta_pos + positions[i - 1]
        positions.append(new_pos)
    """
    
    
    ax_unfiltered.plot(time_series, accs, label=key)
    ax_filtered.plot(times_for_filtered, filtered_accs, label=key)


    print("Accel RMSE: ", get_RMSE(accs))
    print("Average accel: ", sum(accs) / len(accs))
#    print("Filtered Accel RMSE: ", get_RMSE(filtered_accs))
    print()
    

ax_unfiltered.grid()
ax_unfiltered.set_xlabel("Time (s)")
ax_unfiltered.set_ylabel("Acceleration reading (m/s^2)")
ax_unfiltered.set_title("IMU Unfiltered Acceleration over time")
ax_unfiltered.legend()
fig_unfiltered.show()

ax_filtered.grid()
ax_filtered.set_xlabel("Time (s)")
ax_filtered.set_ylabel("Acceleration reading (m/s^2)")
ax_filtered.set_title("IMU Filtered Acceleration over time")
ax_filtered.legend()
fig_filtered.show()






