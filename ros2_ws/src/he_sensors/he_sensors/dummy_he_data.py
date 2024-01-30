import numpy as np
import matplotlib.pyplot as plt

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
    get_ipython().magic('reset -f')
except:
    pass

transition_points_m = [0, 0.1155, 0.231, 0.3465, 0.462, 0.5775, 0.693]
next_transition_idx = 0

velocity = 0.1155
dt = 0.01

x0 = -0.1155
xf = transition_points_m[-1]

t0 = 0
tf = (xf - x0) / velocity
time = np.linspace(t0, tf, int((tf - t0)/dt))

displacement = [x0]

L1_states = [0, 1, 1, 1, 0, 0, 1]
L2_states = [0, 1, 1, 1, 1, 0, 0]
L3_states = [1, 0, 0, 1, 1, 1, 1]
L4_states = [1, 1, 0, 0, 1, 1, 1]

C1_states = [1, 0, 0, 1, 1, 1, 1]
C2_states = [1, 1, 0, 0, 1, 1, 1]
C3_states = [0, 1, 1, 1, 0, 0, 1]
C4_states = [0, 1, 1, 1, 1, 0, 0]

L1_values = []
L2_values = []
L3_values = []
L4_values = []

C1_values = []
C2_values = []
C3_values = []
C4_values = []


for t in range(len(time) - 1):
    
    if next_transition_idx < len(transition_points_m):
        
        displacement.append(velocity * dt + displacement[-1])
        
        L1_values.append(L1_states[next_transition_idx])
        L2_values.append(L2_states[next_transition_idx])
        L3_values.append(L3_states[next_transition_idx])
        L4_values.append(L4_states[next_transition_idx])
        
        C1_values.append(C1_states[next_transition_idx])
        C2_values.append(C2_states[next_transition_idx])
        C3_values.append(C3_states[next_transition_idx])
        C4_values.append(C4_states[next_transition_idx])
        
        if displacement[-1] >= transition_points_m[next_transition_idx]:
            next_transition_idx += 1
            
    
    
plt.grid()
plt.plot(time, displacement, label="Displacement (m)")
plt.plot(time[0:-1], C1_values)
#plt.plot(time[0:-1], C2_values)
#plt.plot(time[0:-1], C3_values)
#plt.plot(time[0:-1], C4_values)

#plt.plot(time[0:-1], L1_values)
#plt.plot(time[0:-1], L2_values)
#plt.plot(time[0:-1], L3_values)
#plt.plot(time[0:-1], L4_values)
plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")

#for tp in transition_points_m: plt.axhline(tp, linestyle='dashed', color='blue')

plt.legend()


# From simulated values, extract state

curr_disp = 0

L1 = []
L2 = []
L3 = []
L4 = []

C1 = []
C2 = []
C3 = []
C4 = []

FS = 0

for t in range(len(time) - 1):
    
    L1.append(L1_values.pop(0))
    L2.append(L2_values.pop(0))
    L3.append(L3_values.pop(0))
    L4.append(L4_values.pop(0))
    
    C1.append(C1_values.pop(0))
    C2.append(C2_values.pop(0))
    C3.append(C3_values.pop(0))
    C4.append(C4_values.pop(0))
    
    if FS < len(L2_states) - 1:
        
        if [L1[-1], L2[-1], L3[-1], L4[-1]] == [L1_states[FS + 1], L2_states[FS + 1], L3_states[FS + 1], L4_states[FS + 1]]:
            print("Next state, ", time[t], transition_points_m[FS])
            FS += 1
        
    

    












