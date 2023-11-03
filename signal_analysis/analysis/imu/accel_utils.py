import matplotlib.pyplot as plt
import numpy as np
   
 
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


def extract_time_series(seconds, nanoseconds):
    time_series = []
    first_time = seconds[0] + nanoseconds[0] / 1000 / 1000 / 1000
    for second, nsec in zip(seconds, nanoseconds):
        
        time_series.append(round(second + nsec / 1000 / 1000 / 1000 - first_time, 3))
        
    return time_series







