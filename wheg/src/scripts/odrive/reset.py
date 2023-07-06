import odrive
from odrive.enums import *

print("connecting...")
Numbers = ["208839824D4D","205839844D4D"]
drv0 = odrive.find_any(serial_number=Numbers[0],timeout=10)
print("connected to", drv0.serial_number)

drv1 = odrive.find_any(serial_number=Numbers[1],timeout=10)
print("connected to", drv1.serial_number)


print("stopping")
drv0.axis0.requested_state = AXIS_STATE_IDLE
drv0.axis1.requested_state = AXIS_STATE_IDLE
drv1.axis0.requested_state = AXIS_STATE_IDLE
drv1.axis1.requested_state = AXIS_STATE_IDLE

odrive.utils.dump_errors(drv0)
odrive.utils.dump_errors(drv1)

i = input("clear errors? (y/n) ")

if i=='y':
    drv0.clear_errors()
    drv1.clear_errors()