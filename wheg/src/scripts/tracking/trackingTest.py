import numpy as np
import odrive
import time
import csv
from odrive.enums import *
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt

GEAR_RATIO_M0 = (60/16)*(60/20)
GEAR_RATIO_M1 = (60/20)*(60/20)

print("reading trajectory")

t_list = []
with open('traj.csv', newline='') as f:
    reader = csv.reader(f,delimiter=',')
    for row in reader:
        arr = np.array(row)
        t_list.append(arr.astype(np.float))
        
phi1_t = t_list[0]
phi2_t = t_list[1]

# fig, ax = plt.subplots(1,1)
# ax.plot(phi1_t)
# fig.savefig('output.svg')

print("connecting to odrive")

drv = odrive.find_any()
odrive.utils.dump_errors(drv)

pos1 = drv.axis0.encoder.pos_estimate
pos2 = drv.axis1.encoder.pos_estimate

print("\nax1:%.4f   ax2:%.4f"%(pos1,pos2))

drv.axis0.requested_state = AXIS_STATE_IDLE
drv.axis1.requested_state = AXIS_STATE_IDLE

drv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
drv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

drv.axis0.controller.config.input_filter_bandwidth = 10
drv.axis1.controller.config.input_filter_bandwidth = 10

drv.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
drv.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER

input("set angles to 0")

drv.axis0.encoder.set_linear_count(0)
drv.axis1.encoder.set_linear_count(0)

start1 = -0.5#phi1_t[0]
start2 = -0.5#phi2_t[0]

pos1 = drv.axis0.encoder.pos_estimate
pos2 = drv.axis1.encoder.pos_estimate

print("\nax1:%.4f   ax2:%.4f"%(pos1,pos2))
drv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
drv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

for j in range(5):
    for i in range(len(phi1_t)):
        p1 = phi1_t[i] - (j)*(np.pi)*0.5 - start1
        p2 = phi2_t[i] - (j)*(np.pi)*0.5 - start2
        
        # p1 = float(input("input phase 1 (rad): "))
        # p2 = float(input("input phase 2 (rad): "))
        
        # convert to motor turns
        m1p1 = p1 * GEAR_RATIO_M1 * (0.5/np.pi)
        m0p2 = p2 * GEAR_RATIO_M0 * (0.5/np.pi)
        
        pos1 = drv.axis0.encoder.pos_estimate
        pos2 = drv.axis1.encoder.pos_estimate
        print("\nax1:%.4f   ax2:%.4f --> p1:%.4f   p2:%.4f"%(pos1,pos2,m0p2,m1p1))
        
        drv.axis0.controller.input_pos = m0p2
        drv.axis1.controller.input_pos = m1p1
        
        err = 1
        # while err < 1e-3:
        #     pos1 = drv.axis0.encoder.pos_estimate
        #     pos2 = drv.axis1.encoder.pos_estimate
        #     err = abs(pos2-m1p1) + abs(pos1-m0p2)
        #     time.sleep(0.01)
        #time.sleep(0.02)
          
set1 = np.floor(drv.axis0.encoder.pos_estimate)
set2 = np.floor(drv.axis1.encoder.pos_estimate)     

drv.axis0.controller.input_pos = set1
drv.axis1.controller.input_pos = set2

print("Idling at 0..")
time.sleep(1)

drv.axis0.encoder.set_linear_count(0)
drv.axis1.encoder.set_linear_count(0)

drv.axis0.requested_state = AXIS_STATE_IDLE
drv.axis1.requested_state = AXIS_STATE_IDLE

