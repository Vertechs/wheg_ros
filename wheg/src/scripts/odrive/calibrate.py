import odrive
from odrive.enums import *
import time
import math

Numbers = ["208839824D4D","205839844D4D"]

for s in Numbers:
    try:
        drv = odrive.find_any(serial_number=s,timeout=20)
    except:
        print("Coudlnt find ", s)
        continue
    
    print("Connected to ", drv.serial_number)
    drv.clear_errors()

    print("Calibrating Axis0")
    drv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    print("Calibrating Axis1")
    drv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    
    while drv.axis1.current_state != AXIS_STATE_IDLE and drv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(.1)
    
    odrive.utils.dump_errors(drv)
    
    pos1 = drv.axis0.encoder.pos_estimate
    pos2 = drv.axis1.encoder.pos_estimate
    
    print("ax1:%.4f   ax2:%.4f"%(pos1,pos2))