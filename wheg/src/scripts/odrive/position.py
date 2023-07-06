import odrive
from odrive.enums import *
import time
import math

Numbers = ["208839824D4D","205839844D4D"]

i = int(input("drive index: "))

print("Connecting")
drv = odrive.find_any(serial_number=Numbers[i])

print("Connected to ", hex(drv.serial_number))

odrive.utils.dump_errors(drv)

print("Position (turns):")
while drv.axis1.current_state == AXIS_STATE_IDLE and drv.axis0.current_state == AXIS_STATE_IDLE:
    
    pos1 = drv.axis0.encoder.pos_estimate
    pos2 = drv.axis1.encoder.pos_estimate
    volt = drv.vbus_voltage
    
    print("ax0:%.4f   ax1:%.4f   V:%.4f\r"%(pos1,pos2,volt), end="")