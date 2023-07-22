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

def controller_loop(axIDs):
    # init ros node
    rospy.init_node("quad_controller")
    clock = rospy.Rate(100)

    # TODO set up subscribers

    # init can bus and load can database
    bus = can.Bus("can0",bustype="socketcan")
    canDB = cantools.database.load_file("configs/odrive-cansimple.dbc")

    # pre-calculate list of can messages to watch for, encoder estimates are 0x09
    posIDs = [a << 5 | canDB.get_message_by_name('Get_Encoder_Estimates').frame_id for a in axIDs]

    # init position estimates array
    n_ax = len(axIDs)
    phi_hat = np.array([float("NaN")]*n_ax, dtype=float)
    phi_rcv = np.array([False]*n_ax, dtype=bool)

    # main control loop
    while not rospy.is_shutdown():
        t0 = time.monotonic()
        phi_rcv.fill(False)

        # get encoder estimates
        while True:
            msg = bus.recv()
            aid = msg.arbitration_id
            for i in range(n_ax):
                if msg.arbitration_id == posIDs[i]:
                    # probably very slow, **REPLACE**
                    phi_hat[i] = canDB.decode_message('Get_Encoder_Estimates',msg.data)['Pos_Estimate']
                    phi_rcv[i] = True
            # break when estimates received for all axes
            if np.all(phi_rcv):
                break

        t1 = time.monotonic()
        rospy.loginfo(t1-t0)


        clock.sleep()



if __name__=='__main__':
    try:
        controller_loop([0xA,0xB,0xC,0xD]) # run loop for axes listed
    except rospy.ROSInterruptException:
        pass