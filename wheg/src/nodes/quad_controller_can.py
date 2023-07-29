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
# TODO, maybe switch to ros timers, send and recieve can on seperate

AXIS_STATE_IDLE = bytearray([1,0,0,0,0,0,0,0])
AXIS_STATE_CLOSED = bytearray([8,0,0,0,0,0,0,0])
CI_VEL_MODE = bytearray([2,0,0,0,1,0,0,0])

def controller_loop(axIDs,bus,canDB):
    # init ros node
    rospy.init_node("quad_controller")
    clock = rospy.Rate(100)

    # TODO set up subscribers

    n_ax = len(axIDs)
    ax_id_offset = int(min(axIDs))
    
    phi_hat = np.array([float("NaN")]*n_ax, dtype=float)
    rx_bytes = [bytearray([0,0,0,0])]*n_ax
    rx_id = [int(0)]*n_ax
    vel_cmd = 0.0
    trq_cmd = 0.0
    
    # initialize lists of CAN message objects for sending, only set data in the control loop
    # set_input_vel = 0x13, ctrl_mode = 0x11, ax_state = 0x07
    vel_msg   = [can.Message(arbitration_id = 0x0D | a<<5, dlc = 8, is_extended_id = False) for a in axIDs]
    mode_msg  = [can.Message(arbitration_id = 0x0B | a<<5, dlc = 8, is_extended_id = False) for a in axIDs]
    state_msg = [can.Message(arbitration_id = 0x07 | a<<5, dlc = 8, is_extended_id = False) for a in axIDs]

    rospy.loginfo("Start axis closed loop control")

    for msg in mode_msg:
        msg.data = CI_VEL_MODE
        bus.send(msg)
    for msg in state_msg:
        msg.data = AXIS_STATE_CLOSED
        bus.send(msg)
        
    # TODO wait for heartbeat response and check errors, must be from other node since can is filtered

    # main control loop
    # while not rospy.is_shutdown():
    for i in range(1000):
        pass
        #t0 = time.monotonic_ns()//1000

        # get encoder estimates, CAN bus is filtered at kernal already
        # so grabbing the next n frames should always give n position packets
        # but in an unknown order. We may lose a frame if an axis with a lower 
        # ID# talks over it but not if all sending at the same frequency
        for i in range(n_ax):
            msg = bus.recv()
            rx_bytes[i] = msg.data[0:4]
            rx_id[i] = msg.arbitration_id
        
        # load estimates into float array, assuming axis IDs are in order
        for i in range(n_ax):
            d = (rx_id[i] >> 5) - ax_id_offset
            phi_hat[d] = struct.unpack('f',rx_bytes[i])[0]

        # do matrix math

        for i in range(n_ax):
            v = phi_hat[i] * -10
            vel_cmd = min(2, max(v, -2))
            trq_cmd = 0
            vel_msg[i].data = struct.pack('f',vel_cmd)+struct.pack('f',trq_cmd)

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in vel_msg:
            bus.send(msg)

        #t1 = time.monotonic_ns()//1000

        clock.sleep()

    rospy.loginfo("Idling")
    # send 0 velocity and torque command to all
    for msg in vel_msg:
        msg.data = 0x0
        bus.send(msg)
    # send idle requested state to all
    for msg in state_msg:
        msg.data = AXIS_STATE_IDLE
        bus.send(msg)

    rospy.loginfo("Exiting")
    bus.shutdown()


if __name__=='__main__':
    
    # init can bus with filters
    filters = [{"can_id":0x009, "can_mask":0x01F, "extended":False}]
    bus_pos_filter = can.interface.Bus(channel="can0",bustype="socketcan",can_filters=filters)
    
    # load can database
    canDB = cantools.database.load_file("configs/odrive-cansimple.dbc")
    
    try:
        controller_loop([0xA,0xB,0xC,0xD],bus_pos_filter,canDB) # run loop for axes listed
    except rospy.ROSInterruptException:
        print("closing can bus" + bus.channel_info)
        bus.shutdown()