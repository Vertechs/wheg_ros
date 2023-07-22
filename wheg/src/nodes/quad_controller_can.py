import can
import cantools
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
import struct

## implementing "decoupled" proportional controller with velocity control signal
## using native interface for setup, state commands, etc
## using CAN for controller inputs and outputs TODO *raise encoder sending frequency
## interfaces appear to be interoperable

## controller and odrive comms run in single node per drive, or all drives??
## node publishes wheel states and subscribes to wheel frame position commands (wheel pos, ext 0->1)
## should also include torque feed forward from inverse kinematics

## TODO all wheels on one node and IK above, or IK and comms in node per wheel?
# node for all four wheels maybe easier to get estimates quickly

# TODO, threadsafe can bus and kernal filtering

def controller_loop(axIDs,bus_pos_filter,canDB):
    # init ros node
    rospy.init_node("quad_controller")
    clock = rospy.Rate(100)

    # TODO set up subscribers

    # pre-calculate list of can messages to watch for, encoder estimates are 0x09
    posIDs = [a << 5 | canDB.get_message_by_name('Get_Encoder_Estimates').frame_id for a in axIDs]

    # init position estimates array
    n_ax = len(axIDs)
    phi_hat = np.array([float("NaN")]*n_ax, dtype=float)
    phi_bytes = [bytearray([0,0,0,0])]*n_ax
    phi_rcv = np.array([False]*n_ax, dtype=bool)

    # main control loop
    while not rospy.is_shutdown():
        t0 = time.monotonic_ns()//1000
        phi_rcv.fill(False)

        # get encoder estimates, CAN bus is filtered at kernal already
        # so grabbing the next n frames should always give n position packets
        # but in an unknown order. may lose a frame if a lower axis talks over it
        for i in range(n_ax):
            phi_bytes[i] = bus_pos_filter.recv().data[0:4]
        
        ## TODO convert to float, do math, send command

        t1 = time.monotonic_ns()//1000
        rospy.loginfo(t1-t0)
        rospy.loginfo(phi_hat)


        clock.sleep()



if __name__=='__main__':
    
    # init can bus with filters
    filters = [{"can_id":0x009, "can_mask":0x01F, "extended":False}]
    bus = can.interface.Bus(channel="can0",bustype="socketcan",can_filters=filters)
    
    # load can database
    canDB = cantools.database.load_file("configs/odrive-cansimple.dbc")
    
    try:
        controller_loop([0xA,0xB,0xC,0xD],bus,canDB) # run loop for axes listed
    except rospy.ROSInterruptException:
        print("closing can bus" + bus.channel_info)
        bus.shutdown()