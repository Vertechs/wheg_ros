#!/usr/bin/env python
import odrive
from odrive.enums import *
import time
import math
import rospy
from std_msgs.msg import float64MultiArray

## Setup and calibrate odrive
## connect with native protocol for better access
## Run node on motor topic to take pos commands

def pos_command(phis):
    # expect 8 long array for all phi values
    rospy.loginfo(rospy.get_caller_id() + "sent to odrive")


def odrive_pos_sub():
    ID =
    rospy.init_node(ID)

    rospy.Subscriber("motors", float64MultiArray, pos_command())

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    drv = odrive.find_any()

    odrive_pos_sub()




print("Connected to ", drv.serial_number)

odrive.utils.dump_errors(drv)

print("Position (turns):")
while drv.axis1.current_state == AXIS_STATE_IDLE and drv.axis0.current_state == AXIS_STATE_IDLE:
    pos1 = drv.axis0.encoder.pos_estimate
    pos2 = drv.axis1.encoder.pos_estimate
    volt = drv.vbus_voltage

    print("ax1:%.4f   ax2:%.4f   V:%.4f\r" % (pos1, pos2, volt), end="")
