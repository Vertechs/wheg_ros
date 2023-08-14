import numpy as np
import odrive
import time
import csv
from odrive.enums import *

## Robot drive indicies
##   (1) O==||==O (2)
##          ||
##          ||
##   (3) O==||==O (4)


GEAR_RATIO = np.array(
    [ [(60/20)*(60/20), (60/20)*(60/20)],
      [(40/14)*(40/14), (40/14)*(40/14)],
      [(40/14)*(40/14), (40/14)*(40/14)],
      [(40/14)*(40/14), (40/14)*(40/14)] ] )

GEAR_RATIO = GEAR_RATIO.flatten()

Numbers = ["208839824D4D","205839844D4D"]

print("reading trajectory")


# get trajectories from table
trajectories = []
with open('traj_simple.csv', newline='') as f:
    reader = csv.reader(f,delimiter=',')
    for row in reader:
        arr = np.array(row)
        trajectories.append(arr.astype(np.float))
        

## First drive
print("connecting to", Numbers[0], " as drv0")

drv0 = odrive.find_any(serial_number=Numbers[0])
odrive.utils.dump_errors(drv0)

pos1 = drv0.axis0.encoder.pos_estimate
pos2 = drv0.axis1.encoder.pos_estimate

print("\nax1:%.4f   ax2:%.4f"%(pos1,pos2))

## Second drive
print("connecting to ", Numbers[1], " as drv1")

drv1 = odrive.find_any(serial_number=Numbers[1])
odrive.utils.dump_errors(drv0)

pos1 = drv1.axis0.encoder.pos_estimate
pos2 = drv1.axis1.encoder.pos_estimate

print("\nax1:%.4f   ax2:%.4f"%(pos1,pos2))


## init control modes
axes = [drv0.axis0,drv0.axis1,drv1.axis0,drv1.axis1]
for axis in axes:
    axis.requested_state = AXIS_STATE_IDLE
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
    
    axis.controller.config.input_filter_bandwidth = 10

input("set angles to 0\n")

# do before closed loop control
for axis in axes:
    axis.encoder.set_linear_count(0)

start = [0.0,0.0,0.0,0.0] ##??

for ax in axes:
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

for j in range(1):
    for i in range(len(trajectories[0])):
        for k in range(len(axes)):
            # get phase and calculate offset
            phi = trajectories[k][i] - (j)*(np.pi)*0.5 - start[k]
    
            # convert to motor turns
            motor_phi = -phi * GEAR_RATIO[k] * (0.5/np.pi)
            
            # send position to drive
            axes[k].controller.input_pos = motor_phi
        
        # display positions
        pos = [x.encoder.pos_estimate for x in axes]
        print("d0_ax0:%.4f   d0_ax1:%.4f   d1_ax0:%.4f   d1_ax1:%.4f\r"%(tuple(pos)),end="")
        time.sleep(0.01)
        
            
# idle all motors
print("idling")
for ax in axes:     
    ax.requested_state = AXIS_STATE_IDLE

