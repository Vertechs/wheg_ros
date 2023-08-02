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

## TODO NOTES ##
# threadsafe can bus and kernal filtering
# maybe switch to ros timers, send and recieve can on seperate
# maybe just switch to C

AXIS_STATE_IDLE = bytearray([1,0,0,0,0,0,0,0])
AXIS_STATE_CLOSED = bytearray([8,0,0,0,0,0,0,0])
CI_VEL_MODE = bytearray([2,0,0,0,1,0,0,0])

AXIS_ID_LIST = [a for a in range(0xA,0x12)]

class PController():
    def __init__(axIDs,pos_filtered_bus,canDB):
        #====Variable Setup====#
        self.bus = pos_filtered_bus
        self.canDB = canDB
        self.axIDs = axIDs
        
        # get the nubmer of axes and the lowest ID, assuming all IDs are grouped
        self.n_ax = len(axIDs)
        self.ax_id_offset = int(min(axIDs))
        
        # control flag set by subscriber on the controller swtich topic
        self.enabled = False
        
        # target position array, in phase and extension amount per wheel 2*n_ax long
        self.targets = [[0.0,0.0]]*n_ax
        self.gains = [[1.0,2.0]]*n_ax
        
        # initialize lists of CAN message objects for sending, only set data in the control loop
        # set_input_vel = 13, ctrl_mode = 11, ax_state = 07
        self.vel_msg = [can.Message(arbitration_id = 0x0D | a<<5, dlc = 8, 
                        is_extended_id = False) for a in axIDs]
        # self.mode_msg = [can.Message(arbitration_id = 0x0B | a<<5, dlc = 8, 
        #                 is_extended_id = False) for a in axIDs]
        # self.state_msg = [can.Message(arbitration_id = 0x07 | a<<5, dlc = 8, 
        #                 is_extended_id = False) for a in axIDs]
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("quad_controller")
        self.clock = rospy.Rate(100)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_status", UInt8MultiArray, switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("run_pos_cmd", Float32MultiArray, set_pos_callback)
    
    
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running (this), 2 Walking, 3 Rolling
        if msg.data[0] == 1:
            self.enabled = True
        else:
            self.enabled = False
            
    def set_pos_callback(self, msg):
        self.targets = msg.data
    
    def controller_loop(self):
        
        # initialize arrays before entering control loop (not sure if actually faster..)
        phi_hat = np.array([float("NaN")]*n_ax, dtype=float)
        rx_bytes = [bytearray([0,0,0,0])]*n_ax
        rx_id = [int(0)]*n_ax
        vel_cmd = 0.0
        trq_cmd = 0.0
        
        while not rospy.is_shutdown():
            if self.enabled:
                self.control_iteration(self,phi_hat,rx_bytes,rx_id,vel_cmd,trq_cmd)
            # must sleep some for subscriber functions to be called
            self.clock.sleep()
            
        rospy.loginfo("shutting down")
        self.bus.shutdown()
        
        ### not needed when using controller switch
        # # send 0 velocity and torque command to all
        # for msg in vel_msg:
        #     msg.data = 0x0
        #     bus.send(msg)
        # # send idle requested state to all
        # for msg in state_msg:
        #     msg.data = AXIS_STATE_IDLE
        #     bus.send(msg)
        
    def control_iteration(self,phi_hat,rx_bytes,rx_id,vel_cmd,trq_cmd):

        # get encoder estimates, CAN bus is filtered at kernal already
        # so grabbing the next n frames should always give n position packets
        # but in an unknown order. We may lose a frame if an axis with a lower 
        # ID# talks over it but rarely if they all send at the same frequency
        for i in range(n_ax):
            msg = self.bus.recv()
            rx_bytes[i] = msg.data[0:4]
            rx_id[i] = msg.arbitration_id
        
        # load estimates into float array, assuming axis IDs are in order
        for i in range(n_ax):
            d = (rx_id[i] >> 5) - ax_id_offset
            phi_hat[d] = struct.unpack('f',rx_bytes[i])[0]

        # calculate controller response, motor phases decoupled into wheel
        # phase and wheel extension so different gains can be used with each
        # TODO use numpy and matrix math to maybe speed up, and merge with above loop?
        for i in range(n_ax):
            v = phi_hat[i] * -10
            # TODO write math hardcoded to test speed difference
            vel_cmd = min(2, max(v, -2))
            trq_cmd = 0
            vel_msg[i].data = struct.pack('f',vel_cmd)+struct.pack('f',trq_cmd)

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in vel_msg:
            self.bus.send(msg)


    


if __name__=='__main__':
    
    # init can bus with filters
    filters = [{"can_id":0x009, "can_mask":0x01F, "extended":False}]
    bus_pos_filter = can.interface.Bus(channel="can0",bustype="socketcan",can_filters=filters)
    
    # load can database TODO remove, no longer used in networking
    canDB = cantools.database.load_file("configs/odrive-cansimple.dbc")
    
    # initialize the controller
    controller = PController(AXIS_ID_LIST, bus_pos_filter, canDB)
    
    try:
        
        controller.controller_loop() # loop will block untill node shutdown
    except rospy.ROSInterruptException:
        # always close bus regardless of exception
        print("closing can bus" + bus.channel_info)
        bus_pos_filter.shutdown()
    except:
        print("non interrupt error, closing can bus")
        bus_pos_filter.shutdown()