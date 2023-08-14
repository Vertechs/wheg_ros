import numpy as np
import odrive
import time
import csv
from odrive.enums import *

## Robot drive indicies
##   (0) O==||==O (1)
##          ||
##          ||
##   (2) O==||==O (3)


GEAR_RATIO = np.array(
    [ [(60/20)*(60/20), (60/20)*(60/20)],
      [(40/14)*(40/14), (40/14)*(40/14)],
      [(40/14)*(40/14), (40/14)*(40/14)],
      [(40/14)*(40/14), (40/14)*(40/14)] ] )

GEAR_RATIO = GEAR_RATIO.flatten()

Numbers = ["208839824D4D","205839844D4D"]

## First drive
print("connecting to", Numbers[0], " as drv0")

drv0 = odrive.find_any(serial_number=Numbers[0])
odrive.utils.dump_errors(drv0)

pos1 = drv0.axis0.encoder.pos_estimate
pos2 = drv0.axis1.encoder.pos_estimate

print("ax1:%.4f   ax2:%.4f"%(pos1,pos2))

## Second drive
print("\nconnecting to ", Numbers[1], " as drv1")

drv1 = odrive.find_any(serial_number=Numbers[1])
odrive.utils.dump_errors(drv0)

pos1 = drv1.axis0.encoder.pos_estimate
pos2 = drv1.axis1.encoder.pos_estimate

print("ax1:%.4f   ax2:%.4f"%(pos1,pos2))


## init control modes
axes = [drv0.axis0,drv0.axis1,drv1.axis0,drv1.axis1]
for axis in axes:
    axis.requested_state = AXIS_STATE_IDLE
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
    
    axis.controller.config.input_filter_bandwidth = 10

input("\nset angles to 0")

# do before closed loop control
for axis in axes:
    axis.encoder.set_linear_count(0)

for ax in axes:
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    
# main loop
go = True
while go:
    # display positions
    pos = np.array([x.encoder.pos_estimate for x in axes])
    whl_pos = np.divide(pos,GEAR_RATIO[0:4])
    print("d0_ax0:%.4f   d0_ax1:%.4f   d1_ax0:%.4f   d1_ax1:%.4f **wheel phase turns**\n"%(tuple(whl_pos)),end="")
    print("Bus voltage: ",drv0.vbus_voltage)
    
    command = input("\nenter relative wheel phases in turns (or quit with q):")
    
    if command == 'q':
        break
    
    try:
        offsets = list(map(float,command.split()))
    except:
        print("invalid")
        continue
    
    if len(offsets) != 4:
        print("need 4 values")
        continue
    
    # update in case of change
    pos = np.array([x.encoder.pos_estimate for x in axes])
    whl_pos = np.divide(pos,GEAR_RATIO[0:4])
    
    targets = whl_pos+np.multiply(offsets,[-1,-1,1,1]) # mirror right phase
    trajectories = np.linspace(whl_pos,targets,100).T
    
    for i in range(len(trajectories[0])):
        for k in range(len(axes)):
            # get phase and calculate offset
            phi = trajectories[k][i]
    
            # convert to motor turns
            motor_phi = phi * GEAR_RATIO[k]
            print(motor_phi,end=" ")
            
            # send position to drive
            axes[k].controller.input_pos = motor_phi
        
        # display positions
        pos = [x.encoder.pos_estimate for x in axes]
        print("\nd0_ax0:%.4f   d0_ax1:%.4f   d1_ax0:%.4f   d1_ax1:%.4f\n"%(tuple(pos)),end="")
        print("Currents: %.4f   %.4f "%(drv0.ibus,drv1.ibus))
        time.sleep(0.01)
        
            
# idle all motors
print("idling")
for ax in axes:     
    ax.requested_state = AXIS_STATE_IDLE

