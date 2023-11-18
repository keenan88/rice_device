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

readings = {}
major_field_strength = 53.26 * np.sin(69.5 * np.pi/180)
minor_field_strength = 53.26 * np.cos(69.5 * np.pi/180)

for minor_ax in ['x', 'y', 'z']:
    
    readings_minor_ax = []
                    
    sampleset = minor_ax + 'east'
    
    print(sampleset)

    try:
        
        mag_readings = db.read_json_file('magnetometer/' + sampleset)
        
        minor_axis_readings = mag_readings['mag'][minor_ax]
        
        avg = round(np.mean(minor_axis_readings[1:]), 2)
        #print(minor_ax + ": " + str(avg))
        
        readings_minor_ax.append(avg)
        
        time_series = extract_time_series(mag_readings['seconds'], mag_readings['nanosecs'])
        
        
        square_sum = 0
        
        fig, ax = plt.subplots()
        
        for dimension, samples in mag_readings['mag'].items():
            
            ax.plot(time_series[1:], samples[1:], label=dimension)
            
            avg = np.mean(samples[1:])
            
            #print(dimension + ":", avg)
            
            if dimension is not minor_ax:
            
                # Assumed that sensor Z is up, sensor Y is north
                if minor_ax == 'x' and dimension == 'z': 
                    scalar = round(major_field_strength / avg, 4)
                    
                    
                elif minor_ax == 'x' and dimension =='y':
                    scalar = round(minor_field_strength / avg, 4)
                    
                # Assumed that sensor Z is up, sensor Y is north
                if minor_ax == 'y' and dimension == 'x': 
                    scalar = round(minor_field_strength / avg, 4)
                    
                elif minor_ax == 'y' and dimension =='z':
                    scalar = round(major_field_strength / avg, 4)
                    
                # Assumed that sensor Z is up, sensor Y is north
                if minor_ax == 'z' and dimension == 'x': 
                    scalar = round(major_field_strength / avg, 4)
                    
                elif minor_ax == 'z' and dimension =='y':
                    scalar = round(minor_field_strength / avg, 4)
                    
                
                print("Scalar ", dimension, scalar)



            
            
        
            
        ax.grid()
        ax.set_xlabel("Time (s)")
        ax.set_ylim(-850, 350)
    
        ax.set_ylabel("Magnetic field (micro teslas)")
        ax.set_title(sampleset + ": Unfiltered, in each dimension over time")
        ax.legend()
        fig.show()
    
    except:
        print("Samplset not available")
        
    print()
                    
                
    readings.update({minor_ax : readings_minor_ax})
                
    

    
    
    



