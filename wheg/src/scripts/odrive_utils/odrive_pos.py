#!/usr/bin/env python
import odrive
from odrive.enums import *
import time
import math
import rospy
from std_msgs.msg import Float32MultiArray, String
import can
import cantools
import os

## publish odrive encoder positions, recieved over CAN bus

pubRate = 10 # publishing frequency (Hz)

def pos_publisher(axIDs = None):
    
    # initialize publisher node
    rospy.loginfo("Starting publisher node")
    pub = rospy.Publisher("encoder_raw", Float32MultiArray, queue_size = 10)
    rospy.init_node("encoder_pub",anonymous=True)
    rate = rospy.Rate(pubRate)
    
    # initialize can bus
    bus = can.Bus("can0",bustype="socketcan")
    rospy.loginfo("CAN bus started: " + bus.channel_info)
    
    # axis CAN ids in order
    if not axIDs:    
        axIDs = [0xA,0xB,0xC,0xD]
    n_ax = len(axIDs)

    # initalize can message database, assuming run from package root
    canDB = cantools.database.load_file("configs/odrive-cansimple.dbc") 
    
    # list of can IDs to watch (encoder estimates should be 0x09)
    posIDs = [a << 5 | canDB.get_message_by_name('Get_Encoder_Estimates').frame_id for a in axIDs]
    
    # initialize data array
    mdata = [float('nan')] * n_ax
    pub_per  = 0.9 / pubRate   # time to spend waiting for CAN messages
    
    rospy.loginfo("Starting publishing loop")
    while not rospy.is_shutdown():
        t0 = time.monotonic()
        
        # scan for CAN messages, exit if running out of time for publishing rate
        while time.monotonic()-t0 < pub_per:
            msg = bus.recv()
            
            # check if arb ID matches encoder position command ID
            if msg.arbitration_id & 0x1F == 0x09:
                
                # check against all axis IDs
                for i in range(n_ax):
                    if axIDs[i] == msg.arbitration_id >> 5:
                        mdata[i] = canDB.decode_message('Get_Encoder_Estimates',msg.data)['Pos_Estimate']
                
        message = Float32MultiArray(data=mdata)
        pub.publish(message)
        
        rospy.loginfo(message)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        pos_publisher()
    except rospy.ROSInterruptException:
        pass
