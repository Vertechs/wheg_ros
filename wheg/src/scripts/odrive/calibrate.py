import odrive
from odrive.enums import *
import time
import math
import configparser
import os.path

config = configparser.ConfigParser()
cf_path = os.path.dirname(os.path.realpath(__file__))+'/../robot_config.ini'
config.read(cf_path)

serial_numbers_str = config.get("Hardware","odrive serial numbers")
serial_numbers = serial_numbers_str.replace(' ','').split(',')

for s in serial_numbers:
    try:
        drv = odrive.find_any(serial_number=s,timeout=20)
    except:
        print("Couldnt find ", s)
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