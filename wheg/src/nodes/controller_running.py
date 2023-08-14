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
# maybe just switch to C..

AXIS_ID_LIST = [a for a in range(0x0A,0x12)]
MAX_VEL = 2.0

class PController():
    def __init__(self,axIDs,pos_filtered_bus,canDB):
        #====Variable Setup====#
        self.bus = pos_filtered_bus
        #self.canDB = canDB # not used
        self.axIDs = axIDs
        
        # get the nubmer of axes and the lowest ID, assuming all IDs are grouped
        self.n_ax = len(axIDs)
        self.ax_id_offset = int(min(axIDs))
        
        # control flag set by subscriber on the controller swtich topic
        self.enabled = False
        
        # target position array, in phase and extension amount per wheel 2*n_drives long
        self.targets = [0.0,0.0]*(self.n_ax//2)
        self.gains = [-1.0,-2.0]*(self.n_ax//2) #TODO what is this??
        
        # initialize lists of CAN message objects for sending, only set data in the control loop
        # set_input_vel = 13, ctrl_mode = 11, ax_state = 07
        self.vel_msg = [can.Message(arbitration_id = 0x0D | a<<5, dlc = 8, 
                        is_extended_id = False) for a in axIDs]
        self.pos_req = [can.Message(arbitration_id = 0x09 | a<<5, dlc = 8,
                        is_extended_id=False, is_remote_frame = True) for a in axIDs]
        # self.mode_msg = [can.Message(arbitration_id = 0x0B | a<<5, dlc = 8, 
        #                 is_extended_id = False) for a in axIDs]
        # self.state_msg = [can.Message(arbitration_id = 0x07 | a<<5, dlc = 8, 
        #                 is_extended_id = False) for a in axIDs]
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("run_controller")
        self.clock = rospy.Rate(1)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_mode", UInt8MultiArray, self.switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("run_pos_cmd", Float32MultiArray, self.set_pos_callback)
    
    
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running (this), 2 Walking, 3 Rolling
        if msg.data[0] == 1:
            if not self.enabled:
                self.enabled = True
                rospy.loginfo("Run mode enabled")
        else:
            if self.enabled:
                self.enabled = False
                rospy.loginfo("Run mode disabled")
            
    def set_pos_callback(self, msg):
        self.targets = msg.data
    
    def controller_loop(self):
        
        # initialize arrays before entering control loop (not sure if actually faster..)
        self.phi_hat = np.array([float("NaN")]*self.n_ax, dtype=float)
        self.rx_bytes = [bytearray([0,0,0,0])]*self.n_ax
        self.rx_id = [int(0)]*self.n_ax
        self.vel_cmd = [0.0]*self.n_ax
        self.trq_cmd = [0.0]*self.n_ax
        
        while not rospy.is_shutdown():
            t0 = time.monotonic_ns()
            if self.enabled:
                self.control_iteration_request()
            t1 = time.monotonic_ns()
            tms = (t1-t0)*0.000001
            rospy.loginfo("iter took %.2f ms"%tms)
            rospy.loginfo("estimates:"+str([round(p, 3) for p in self.phi_hat]))
            # must sleep some for subscriber functions to be called
            self.clock.sleep()
            
        rospy.loginfo("shutting down")
        self.bus.shutdown()
        
    
    def control_iteration_request(self):
        # """
        # Getting estimates from request frames sent by main board
        # Slower than cyclic but potentially more consistent
        # """
        for i in range(self.n_ax):
            self.bus.send(self.pos_req[i])
            self.rx_bytes[i] = self.bus.recv(timeout=1).data[0:4]
            self.phi_hat[i] = struct.unpack('f',self.rx_bytes[i])[0]
            
        self.control_math()
        
        for i in range(self.n_ax):
            self.vel_msg[i].data = struct.pack('f',self.vel_cmd[i])+struct.pack('f',self.trq_cmd[i])
        
        for msg in self.vel_msg:
            self.bus.send(msg)
            
            
    def control_iteration_cyclic(self):
        # """
        # Getting estimates from cyclic messages sent from drives
        # Faster than sending rtr frame but may occasionally miss frames
        # Operations in seperate loops to be sure we dont miss any frames

        # CAN bus is filtered at kernal already so grabbing
        # the next n frames should always give an estiamte from each drive
        # but in an unknown order. May lose a frame if an axis with a lower 
        # ID# talks over it but rarely as they all send at the same frequency
        # """
        
        for i in range(self.n_ax):
            frame = self.bus.recv()
            self.rx_bytes[i] = frame.data[0:4]
            self.rx_id[i] = frame.arbitration_id
        
        # load estimates into float array, assuming axis IDs are in order
        for i in range(self.n_ax):
            d = (self.rx_id[i] >> 5) - self.ax_id_offset
            self.phi_hat[d] = struct.unpack('f',self.rx_bytes[i])[0]

        self.control_math()
        
        for i in range(self.n_ax):
            self.vel_msg[i].data = struct.pack('f',self.vel_cmd[i])+struct.pack('f',self.trq_cmd[i])

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in self.vel_msg:
            self.bus.send(msg)
            
    def control_math(self):
        # """
        # calculate controller response, motor phases decoupled into wheel
        # phase and wheel extension so different gains can be used with each
        # TODO test numpy matrix math vs per drive to see if faster
        # """
        for i in range(self.n_ax//2):
            self.vel_cmd = (self.phi_hat[i]-self.targets[i]) * self.gains[i]
            self.vel_cmd = min(MAX_VEL, max(self.vel_cmd, -MAX_VEL)) # clamp velocity
            self.trq_cmd = 0 # can get from IK


    


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