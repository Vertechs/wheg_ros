import can
import rospy
import cantools
import time
import numpy as np
import struct
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

# local package, install as editable
from wheg_utils.four_bar_wheg_arms import WhegFourBar
from wheg_utils import robot_config
robot = robot_config.get_config_A()

## Implementing position controller governed by CPG
# using some IK math to determine actual motor position from target 
# extension and phase, and torque feedforward term based on quasi-statics

# populate axis IDs, 10 to 18 by default
AXIS_ID_LIST = [a for a in range(0xA,0x12)]

# convert from radians to rotations for odrive controller
CIRC = (0.5/np.pi)

class PController(can.Listener):
    def __init__(self,axIDs,pos_filtered_bus):
        #====Variable Setup====#
        self.bus = pos_filtered_bus
        # self.canDB = canDB
        self.axIDs = axIDs
        
        # get the nubmer of axes and the lowest ID, assuming all IDs are grouped
        self.n_ax = len(axIDs)
        self.n_whl = self.n_ax//2
        self.ax_id_offset = int(min(axIDs))
        
        # control flag set by subscriber on the controller swtich topic
        self.enabled = False
        
        # target position arrays, rotational and extension position targets
        self.phi_tar = [0.0]*self.n_ax
        self.rot_tar = [0.0]*(self.n_whl)
        self.ext_tar = [0.0]*(self.n_whl)
        
        # state estiamtes from odrive, passed back to cpg
        self.phi_hat = [0.0]*self.n_ax
        self.rot_hat = [0.0]*(self.n_whl)
        self.ext_hat = [0.0]*(self.n_whl)
        
        # feed forward terms
        self.vel_ff = [0]*self.n_ax
        self.trq_ff = [0]*self.n_ax # feed forward must be int, taken as x*10^-3 in odrive

        # intermediate terms used in control math
        self.e,self.p,self.h1,self.h2 = 0.0,0.0,0.0,0.0
        
        #===CAN setup===#
        
        # initialize lists of CAN message objects for sending, only set data in the control loop
        self.pos_msg = [can.Message(arbitration_id = 0x0C | a<<5, dlc = 8, 
                        is_extended_id = False) for a in self.axIDs]
                        
        self.pos_arb_ids = [0x09 | a<<5 for a in axIDs]
        self.pos_req = [can.Message(arbitration_id = a, dlc = 8,
                        is_extended_id=False, is_remote_frame = True) for a in self.pos_arb_ids]
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("walk_controller")
        # pos sending rate. set point is filtered in odrive, make sure bandwidth in
        # controller_switch is ~0.5x the sending rate
        self.clock = rospy.Rate(50)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_mode", UInt8MultiArray, self.switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("walk_pos_cmd", Float32MultiArray, self.target_callback)
        self.pos_estimate_publisher = rospy.Publisher("walk_pos_est", Float32MultiArray, queue_size=2)
        
        #===Wheg setup===#
        # initialize wheg objects for each wheel module
        self.whegs = []
        for i in range(self.n_whl):
            self.whegs.append(WhegFourBar(robot.modules[i].four_bar.get_parameter_list()))
        
        # directions depending on motor location and wheel orientation #TODO move to config
        self.ext_dir = [1,1,1,-1] # extension direction for each wheel s.t. expanding is +
        self.rot_dir = [-1,1,-1,1]  # rotation direction for each wheel s.t. forward is +
            
        # load gear ratios for drive
        self.inner_ratio = [mod.inner_ratio for mod in robot.modules]
        self.outer_ratio = [mod.outer_ratio for mod in robot.modules]
        self.ext_ratio = [mod.whl_ratio for mod in robot.modules]
        print(self.ext_ratio)
        
        rospy.loginfo("Walking controller launched")
    
    def on_message_received(self, msg: can.Message) -> None:
        # CAN message callback used with notifier thread
        # Only function required to be registered with canbus notifier
        # Set internal pos estimates whenever estimate frame recieved
        # takes ~100us to run; TODO may be too slow, can process outside of call
        for i in range(self.n_ax):
            if msg.arbitration_id == self.pos_arb_ids[i]:
                self.phi_hat[i] = struct.unpack('f',msg.data[0:4])
    
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running , 2 Walking, 3 Rolling (this)
        if msg.data[0] == 2:
            if not self.enabled:
                rospy.loginfo("Walk controller enabled")
                self.enabled = True
        else:
            if self.enabled:
                self.enabled = False
                rospy.loginfo("Walk controller disabled")
            
    def target_callback(self, msg):
        # set target positions from message data
        self.rot_tar = msg.data[0:4]
        self.ext_tar = msg.data[4:8]
    
    def controller_loop(self):
        self.e = 0.0
        self.p = 0.0
        
        while not rospy.is_shutdown():
            if self.enabled:
                # t1 = time.monotonic_ns()

                self.send_estimates()
                self.control_math()
                self.send_commands()
                
                # t2 = time.monotonic_ns()
                # rospy.loginfo("Loop took %d us"%( (t2-t1) // 1000) )
            # must sleep some for subscriber functions to be called
            self.clock.sleep()
            
        # exit cleanup, shutdown CAN bus
        rospy.loginfo("shutting down")
        self.bus.shutdown()
        
    def control_math(self):
        for i in range(self.n_whl):
            # calculate motor phases from gearing ratios
            self.e = (self.ext_tar[i] * self.ext_ratio[i]) * self.ext_dir[i] * 0.5
            self.p = self.rot_tar[i] * self.rot_dir[i]
            self.phi_tar[2*i] = self.outer_ratio[i] * (self.p - self.e) * CIRC
            self.phi_tar[2*i+1] = self.inner_ratio[i] * (self.p + self.e) * CIRC
            
            #self.trq_ff[2*i:2*i+1] = self.wheg.IK_f(self.phi_hat[..]) #TODO get from IK wheg object
        
    def send_commands(self):
        # calculate desired positions
        for i in range(self.n_ax):
            # set message byte arrays with position and velocity and torque feed forward
            self.pos_msg[i].data = struct.pack('f',self.phi_tar[i])+struct.pack('h',self.vel_ff[i])+struct.pack('h',self.trq_ff[i])

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in self.pos_msg:
            self.bus.send(msg)

    def send_estimates(self):
        # inverse control math to get wheel phases and extensions
        for i in range(self.n_whl):
            self.h1 = self.phi_hat[2*i] / (self.outer_ratio[i]*CIRC)
            self.h2 = self.phi_hat[2*i+1] / (self.inner_ratio[i]*CIRC)
            self.p = (self.h1 + self.h2) * 0.5
            self.e = (self.h2 - self.h1) * 0.5
            self.ext_hat = self.e * 2 * self.ext_dir[i] / self.ext_ratio[i]
            self.rot_hat = self.p * self.rot_dir[i]




if __name__=='__main__':
    
    # init can bus with filters
    filters = [{"can_id":0x009, "can_mask":0x01F, "extended":False}]
    bus_pos_filter = can.ThreadSafeBus(channel="can0",bustype="socketcan",can_filters=filters)
    
    # initialize the controller
    controller = PController(AXIS_ID_LIST, bus_pos_filter)
    
    try:
        controller.controller_loop() # loop will block untill node shutdown
    # always close bus regardless of exception
    except rospy.ROSInterruptException:
        print("closing can bus" + bus_pos_filter.channel_info)
        bus_pos_filter.shutdown()