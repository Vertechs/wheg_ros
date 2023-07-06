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

Numbers = ["208839824D4D","205839844D4D"]

## First drive
print("connecting to", Numbers[0], " as drv0")

drv0 = odrive.find_any(serial_number=Numbers[0])
odrive.utils.dump_errors(drv0)

## Second drive
print("\nconnecting to ", Numbers[1], " as drv1")

drv1 = odrive.find_any(serial_number=Numbers[1])
odrive.utils.dump_errors(drv0)


## init control modes
axes = [drv0.axis0,drv0.axis1,drv1.axis0,drv1.axis1]
for axis in axes:
    axis.requested_state = AXIS_STATE_IDLE
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
    
    axis.controller.config.input_filter_bandwidth = 10

input("\nset angles to 0, enter closed loop control")

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
    
    args = command.split()
    try:
        letter = args[0]
        gains = list(map(float,args[1,4]))
    except:
        print("invalid")
        continue
    
    if len(offsets) != 4:
        print("need 4 values")
        continue
    
    if letter == 'v'
        for ax in axes:
            ax.controller.config.vel_gain = args.pop(0)
    elif letter == 'p'
        for ax in axes:
            ax.controller.config.pos_gain = args.pop(0)
    
    elif letter == 'i'
        for ax in axes:
            ax.controller.config.vel_integrator_gain = args.pop(0)
    
    else
        print('Unrecognized ', letter)
        
            
# idle all motors
print("idling")
for ax in axes:     
    ax.requested_state = AXIS_STATE_IDLE
