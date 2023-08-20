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

AXIS_STATE_IDLE = bytearray([1,0,0,0,0,0,0,0])
AXIS_STATE_CLOSED = bytearray([8,0,0,0,0,0,0,0])
CI_VEL_MODE = bytearray([2,0,0,0,1,0,0,0])

# populate axis IDs, 10 to 18 by default
AXIS_ID_LIST = [a for a in range(0xA,0x12)]

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
        self.clock = rospy.Rate(10)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_mode", UInt8MultiArray, self.switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("walk_pos_cmd", Float32MultiArray, self.target_callback)
        
        #===Wheg setup===#
        # initialize wheg objects for each wheel module
        self.whegs = []
        for i in range(self.n_whl):
            self.whegs.append(WhegFourBar(robot.modules[i].four_bar.get_parameter_list()))
            
        # calculate extension phase difference including wheel internal gearing
        # convert to turn from radians
        self.ext_multiplier = []
        for i in range(self.n_whl):
            mod = robot.modules[i]
            ratio = mod.ext_phase_diff * mod.whl_ratio * (0.5/np.pi)
            self.ext_multiplier.append(ratio)
            
        # load gear ratios for drive
        self.inner_ratio = [mod.inner_ratio for mod in robot.modules]
        self.outer_ratio = [mod.outer_ratio for mod in robot.modules]
        
        # direction for deployment 
        # (+1 for +ext ~= outer - inner | -1 for +ext ~= -outer + inner)
        self.whl_dir = [1,-1,-1,1]
        
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
            # calculate difference in hub phase needed
            self.e = self.ext_tar[i] * self.ext_multiplier[i] * 0.5 * self.whl_dir[i]
            
            # calculate target phase for each *motor* using IK or linear approx
            self.phi_tar[2*i] = self.outer_ratio[i] * (self.rot_tar[i] + self.e)
            self.phi_tar[2*i+1] = self.inner_ratio[i] * (self.rot_tar[i] - self.e)
            
            #self.trq_ff[2*i:2*i+1] = self.wheg.IK_f(self.phi_hat[..]) #TODO get from IK wheg object
        
    def send_commands(self):
        # calculate desired positions
        for i in range(self.n_ax):
            # set message byte arrays with position and velocity and torque feed forward
            self.pos_msg[i].data = struct.pack('f',self.phi_tar[i])+struct.pack('h',self.vel_ff[i])+struct.pack('h',self.trq_ff[i])

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in self.pos_msg:
            self.bus.send(msg)


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
        print("closing can bus" + bus.channel_info)
        bus_pos_filter.shutdown()