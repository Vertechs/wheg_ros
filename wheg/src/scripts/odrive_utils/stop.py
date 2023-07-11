import odrive
from odrive.enums import *

print("connecting to odrive")

drv = odrive.find_any()

print("stopping")
drv.axis0.requested_state = AXIS_STATE_IDLE
drv.axis1.requested_state = AXIS_STATE_IDLE
odrive.utils.dump_errors(drv)

i = input("clear errors? (y/n) ")

if i=='y':
    drv.clear_errors()