import odrive
from odrive.enums import *

print("connecting...")
Numbers = ["208839824D4D","205839844D4D", "209039854D4D", "205239824D4D"]
drives = []

# connect to drives, stop axes, and append to list
for sn in Numbers:
    drv = odrive.find_any(serial_number=sn,timeout=10)
    print("connected to", drv.serial_number)
    drv.axis0.requested_state = AXIS_STATE_IDLE
    drv.axis1.requested_state = AXIS_STATE_IDLE
    drives.append(drv)

# clear errors one at a time
for drv in drives:
    odrive.utils.dump_errors(drv)
    i = input("clear errors for %s? (y/n) "%drv.serial_number)
    if i=='y':
        drv.clear_errors()