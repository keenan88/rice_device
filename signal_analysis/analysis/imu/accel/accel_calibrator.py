import matplotlib.pyplot as plt
import numpy as np
import DB_utils as db
import accel_utils as au

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
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



for sampleset in ['neg_z_on_table']: #, 'neg_x_on_table', 'neg_y_on_table'

    print(sampleset)
    imu_readings = db.read_json_file('accelerometer/' + sampleset)
    
    time_series = au.extract_time_series(imu_readings['seconds'], imu_readings['nanosecs'])
    
    for datatype in ['accel', 'gyro']:
    
        readings = imu_readings[datatype]
        
        square_sum = 0
        
        fig, ax = plt.subplots()
        
        for dimension, samples in readings.items():
            
            ax.plot(time_series, samples, label=dimension)
            
            print(datatype + " " + dimension + ":", np.mean(samples))
            
            #square_sum += np.mean(np.square(samples))
            
        
        ax.grid()
        ax.set_xlabel("Time (s)")
        if datatype == 'accel':
            ax.set_ylabel("Acceleration reading (m/s^2)")
        elif datatype == 'gyro':
            ax.set_ylabel("Rotational velocity (rad/s)")
        ax.set_title(sampleset + ": Unfiltered " + datatype + " In each dimension over time")
        ax.legend()
        fig.show()
        
    print()
    
    
    



