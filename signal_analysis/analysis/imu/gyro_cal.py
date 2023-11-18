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

def trapezoidal_integration(data, time_scale):
    time_scale = list(time_scale)

    step_size = (time_scale[-1] - time_scale[0]) / (len(time_scale) - 1)

    result = [0.0] * len(time_scale)

    for i in range(1, len(time_scale)):
        result[i] = result[i - 1] + 0.5 * (data[i - 1] + data[i]) * step_size

    return result




for sampleset in ['neg_z_on_table']: #, 'neg_x_on_table', 'neg_y_on_table'

    print(sampleset)
    imu_readings = db.read_json_file('imu/' + sampleset)
    
    time_series = au.extract_time_series(imu_readings['seconds'], imu_readings['nanosecs'])
    
    readings = imu_readings['gyro']
        
    fig, ax = plt.subplots()
    
    for dimension, samples in readings.items():
        
        ax.plot(time_series, samples, label=dimension)
        
        print("Gyro" + " " + dimension + ":", np.mean(samples))
        
    
    ax.grid()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rotational velocity (rad/s)")
    ax.set_title(sampleset + ": Unfiltered gyroscope readings In each dimension over time")
    ax.legend()
    fig.show()
    
    fig, ax = plt.subplots()
    for dimension, samples in readings.items():
        
        scaled_samples = 0.0011772910308420256 * np.array(samples)
        
        rot_disp = trapezoidal_integration(scaled_samples, time_series)
        ax.plot(time_series, rot_disp, label=dimension)
        
        if dimension == 'y':
            saved = trapezoidal_integration(scaled_samples, time_series)
        
        
    
    ax.grid()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rotational displacement (rads)")
    ax.set_title(sampleset + ": Unfiltered trapezoidally integrated gyroscope readings In each dimension over time")
    ax.legend()
    fig.show()
    
    print()
    
    
    



