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

drv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
drv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

drv.axis0.controller.config.vel_ramp_rate = 0.5
drv.axis1.controller.config.vel_ramp_rate = 0.5

drv.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
drv.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

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

drv.axis0.controller.input_vel = -5
drv.axis1.controller.input_vel = -5

time.sleep(20)

drv.axis0.controller.input_vel = 0.0
drv.axis1.controller.input_vel = 0.0

time.sleep(10)

drv.axis0.requested_state = AXIS_STATE_IDLE
drv.axis1.requested_state = AXIS_STATE_IDLE

