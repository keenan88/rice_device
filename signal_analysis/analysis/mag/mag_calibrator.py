import matplotlib.pyplot as plt
import numpy as np
import DB_utils as db
#import accel_utils as au

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
    get_ipython().magic('reset -f')
except:
    pass

def extract_time_series(seconds, nanoseconds):
    time_series = []
    first_time = seconds[0] + nanoseconds[0] / 1000 / 1000 / 1000
    for second, nsec in zip(seconds, nanoseconds):
        
        time_series.append(round(second + nsec / 1000 / 1000 / 1000 - first_time, 3))
        
    return time_series
    
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



for sampleset in ['x_south', 'x_north', 'y_south', 'y_north', 'z_south', 'z_north']:

    print(sampleset)
    mag_readings = db.read_json_file('magnetometer/' + sampleset)
    
    time_series = extract_time_series(mag_readings['seconds'], mag_readings['nanosecs'])
    
    square_sum = 0
    
    fig, ax = plt.subplots()
    
    for dimension, samples in mag_readings['mag'].items():
        
        ax.plot(time_series[1:], samples[1:], label=dimension)
        
        print(dimension + ":", np.mean(samples[1:]))
        
        #square_sum += np.mean(np.square(samples))
        
    ax.grid()
    ax.set_xlabel("Time (s)")

    ax.set_ylabel("Magnetic field (micro teslas)")
    ax.set_title(sampleset + ": Unfiltered, in each dimension over time")
    ax.legend()
    fig.show()
        
    print()
    
    
    



