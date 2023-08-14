import can
import rospy
import cantools
import time
import numpy as np
import struct
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

# local package, install as editable
from wheg_utils.wheg_4bar import WhegFourBar
from wheg_utils import robot_config
robot = robot_config.get_config_A()

## implementing full IK position control
# input: footfall locations relative to robot COM
# and integer value for arc to use (increasing to move forward) 


AXIS_STATE_IDLE = bytearray([1,0,0,0,0,0,0,0])
AXIS_STATE_CLOSED = bytearray([8,0,0,0,0,0,0,0])
CI_VEL_MODE = bytearray([2,0,0,0,1,0,0,0])

# populate axis IDs, 10 to 18 by default
AXIS_ID_LIST = [a for a in range(0xA,0x12)]

# TODO should load from config
WHEEL_CIRC = 0.07 * 2 * 3.1415 # in meters, angular pos measured in revolutions
WHEEL_DIST = 0.19 # horizontal distance between wheels in meters
RAD_TO_CIRC = 1.0 / (np.pi * 2)

class PController():
    def __init__(self,axIDs,pos_filtered_bus):
        #====Variable Setup====#
        self.bus = pos_filtered_bus
        # self.canDB = canDB
        self.axIDs = axIDs
        
        # get the nubmer of axes and the lowest ID, assuming all IDs are grouped
        self.n_ax = len(axIDs)
        self.ax_id_offset = int(min(axIDs))
        
        # control flag set by subscriber on the controller swtich topic
        self.enabled = False
        
        # target position arrays, rotational and extension position
        self.rot = [0.0]*(self.n_ax//2)
        self.ext = [0.0]*(self.n_ax//2)
        
        # initialize lists of CAN message objects for sending, only set data in the control loop
        self.pos_msg = [can.Message(arbitration_id = 0x0C | a<<5, dlc = 8, 
                        is_extended_id = False) for a in self.axIDs]
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("walk_controller")
        self.clock = rospy.Rate(10)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_status", UInt8MultiArray, self.switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("walk_pos_cmd", Float32MultiArray, self.pos_callback)
        
        rospy.loginfo("pos test controller started")
    
    
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running , 2 Walking, 3 Rolling (this)
        if msg.data[0] == 2:
            if not self.enabled:
                rospy.loginfo("Enabled")
                self.enabled = True
        else:
            if self.enabled:
                self.enabled = False
                rospy.loginfo("Disabled")
            
    def pos_callback(self, msg):
        self.rot = msg.data[0:4]
        self.ext = msg.data[4:8]
    
    def controller_loop(self):
        
        # initialize arrays before entering control loop (not sure if actually faster..)
        self.rx_bytes = [bytearray([0,0,0,0])]*self.n_ax
        self.rx_id = [int(0)]*self.n_ax
        self.phi0 = 0.0
        self.phi1 = 0.0
        self.vel_ff = 0
        self.trq_ff = 0 # feed forward must be int, taken as x*10^-3 in odrive 
        
        while not rospy.is_shutdown():
            if self.enabled:
                t1 = time.monotonic_ns()
                self.control_iteration()
                t2 = time.monotonic_ns()
                rospy.loginfo("Loop took %d"%( (t2-t1) // 1000) )
            # must sleep some for subscriber functions to be called
            self.clock.sleep()
            
        # exit cleanup, shutdown CAN bus
        rospy.loginfo("shutting down")
        self.bus.shutdown()
        
        
    def control_iteration(self):
        
        # calculate desired positions
        for i in range(self.n_ax//2):
            if i%2 == 0: # odd wheels on right, ccw(+) = forward
                d = -1
            else:
                d = 1
                
            if i<2: # opposite of d on back, same in front
                s = 1
            else:
                s = -1
            
            # TODO replace with call to wheg object for position and torque
            self.phi0 = RAD_TO_CIRC * np.pi * d*self.rot[i]
            self.phi1 = RAD_TO_CIRC * np.pi * d*self.rot[i] + s*d*self.ext[i]
            
            # set message byte arrays with position and velocity and torque feed forward
            self.pos_msg[2*i].data   = struct.pack('f',self.phi0)+struct.pack('h',self.vel_ff)+struct.pack('h',self.trq_ff)
            self.pos_msg[2*i+1].data = struct.pack('f',self.phi1)+struct.pack('h',self.vel_ff)+struct.pack('h',self.trq_ff)

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in self.pos_msg:
            self.bus.send(msg)


if __name__=='__main__':
    
    # init can bus with filters
    filters = [{"can_id":0x009, "can_mask":0x01F, "extended":False}]
    bus_pos_filter = can.interface.Bus(channel="can0",bustype="socketcan",can_filters=filters)
    
    # initialize the controller
    controller = PController(AXIS_ID_LIST, bus_pos_filter)
    
    try:
        controller.controller_loop() # loop will block untill node shutdown
    # always close bus regardless of exception
    except rospy.ROSInterruptException:
        print("closing can bus" + bus.channel_info)
        bus_pos_filter.shutdown()