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

class PController(can.Listener):
    def __init__(self,axIDs,pos_filtered_bus,canDB):
        super().__init__()
        
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
        self.phi_gain = [1.0]*(self.n_ax//2)
        self.ext_gain = [4.0]*(self.n_ax//2)
        

        # == CAN SETUP == #
        # initialize lists of CAN message objects for sending, only set data in the control loop
        # set_input_vel = 13, ctrl_mode = 11, ax_state = 07
        self.vel_msg = [can.Message(arbitration_id = 0x0D | a<<5, dlc = 8, 
                        is_extended_id = False) for a in axIDs]
        
        self.pos_arb_ids = [0x09 | a<<5 for a in axIDs]
        self.pos_req = [can.Message(arbitration_id = a, dlc = 8,
                        is_extended_id=False, is_remote_frame = True) for a in self.pos_arb_ids]

        # initialize arrays before entering control loop
        self.phi_est = np.array([float("NaN")]*self.n_ax, dtype=float)
        self.rx_bytes = [bytearray([0,0,0,0])]*self.n_ax
        self.rx_id = [int(0)]*self.n_ax
        self.vel_cmd = [0.0]*self.n_ax
        self.trq_cmd = [0.0]*self.n_ax
        self.buffer_ind = 0
        
        self.make_requests = False
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("run_controller")
        self.clock = rospy.Rate(100)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_mode", UInt8MultiArray, self.switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("run_pos_cmd", Float32MultiArray, self.set_tar_callback)
        
        rospy.loginfo("Run controller launched")
        
    
    #===Networking callback functions===#
    
    def on_message_received(self, msg: can.Message) -> None:
        # CAN message callback used with notifier thread
        # Only function required to be registered with canbus notifier
        # Set internal buffer whenever estimate frame recieved
        # frames should always be in some order so a buffer of n frames will have
        # n different encoder messages, though the exact order will be unknown
        # takes ~50us to run this
        print(msg)
        self.rx_id[self.buffer_ind] = msg.arbitration_id
        self.rx_bytes[self.buffer_ind] = msg.data[0:4]
        if self.buffer_ind >= self.n_ax-1:
            self.buffer_ind = 0
        else:
            self.buffer_ind += 1
            
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running (this), 2 Walking, 3 Rolling, 4 running (requests)
        if msg.data[0] == 1 or msg.data[0]==4:
            if not self.enabled:
                self.enabled = True
                rospy.loginfo("Run mode enabled")
                
        else:
            if self.enabled:
                self.enabled = False
                rospy.loginfo("Run mode disabled")
                
        # CAN mode is second int of status array
        # 0 cyclic messages, 1 request and response
        if msg.data[1] == 0:
            self.make_requests = False
        else:
            self.make_requests = True
            
    
    def set_tar_callback(self, msg):
        self.targets = msg.data # set target pos from ROS topic
        
        
    #===Controller functions===#
    
    def controller_loop(self):
        # control logic, without networking takes ~5ms to run
        # interrupts from the callback functions push to ~10ms with significant deviation
        # using rtr frames is more steady around ~15ms
        t0 = time.monotonic_ns()
        
        # variables for the control math function
        self._i0 = 1
        self._i1 = 1
        self._p = 0.0
        self._e = 0.0
        self._v0 = 0.0
        self._v1 = 0.0
        
        while not rospy.is_shutdown():
            if self.enabled:
                # only request encoder frames if not receiving cyclic messages
                if self.make_requests:
                    self.request_estimates()
                
                self.update_estimates()
                self.control_math()
                self.send_commands()
                
                # t1 = time.monotonic_ns()
                # tms = (t1-t0)*0.000001
                
                # rospy.loginfo("iter took %.2f ms"%tms)
                # rospy.loginfo("estimates: "+str([round(p, 3) for p in self.phi_est]))
                # rospy.loginfo("commands:  "+str([round(p, 3) for p in self.vel_cmd]))
                
                # t0 = time.monotonic_ns()

            
            # must sleep some for ROS subscriber functions to be called
            self.clock.sleep()
            
        rospy.loginfo("shutting down")
        self.bus.shutdown()
        
    def request_estimates(self):
        # """
        # Getting estimates from request frames sent by main board
        # Slower than cyclic but potentially more consistent
        # notifier thread calls our on_message_recieved function for both
        #
        # Getting estimates from cyclic messages sent from drives is
        # faster than sending rtr frame but encoder estimates will not 
        # all be taken at the same time. Frames may also be lost when
        # an axis talks over another and wins arbitration. Should be rare
        # when they are sending at the same frequency.
        # """
        
        for i in range(self.n_ax):
            self.bus.send(self.pos_req[i])
            
    def update_estimates(self):
        # find where each recieved message goes and translate to float
        # not done in on_message_recieved in order to keep that callback fast
        for i in range(self.n_ax):
            for j in range(self.n_ax):
                if self.rx_id[i] == self.pos_arb_ids[j]:
                    self.phi_est[j] = struct.unpack('f',self.rx_bytes[i])[0]
                    
    def send_commands(self):
        # translate controller responses into can messages
        for i in range(self.n_ax):
            self.vel_msg[i].data = struct.pack('f',self.vel_cmd[i])+struct.pack('f',self.trq_cmd[i])
            
        # send can messages as fast as possible, will lose arbitration to encoder frames
        for msg in self.vel_msg:
            self.bus.send(msg)
            
    def control_math(self):
        # """
        # calculate controller response, motor phases decoupled into wheel
        # phase and wheel extension so different gains can be used with each
        # TODO test numpy matrix math vs per drive to see if faster
        # """
        for i in range(self.n_ax//2):
            self._i0 = i*2
            self._i1 = i*2+1
            self._e = (self.phi_est[self._i0] - self.phi_est[self._i1]) * self.ext_gain[i]
            self._p = (self.phi_est[self._i0] + self.phi_est[self._i1]) * self.phi_gain[i]
            self._v0 = - self._p - self._e
            self._v1 = - self._p + self._e
            self.vel_cmd[self._i0] = min(MAX_VEL, max(self._v0, -MAX_VEL)) # clamp velocity
            self.vel_cmd[self._i1] = min(MAX_VEL, max(self._v1, -MAX_VEL)) # clamp velocity


if __name__=='__main__':
    
    # init can bus with filters
    filters = [{"can_id":0x009, "can_mask":0x01F, "extended":False}]
    bus_pos_filter = can.ThreadSafeBus(channel="can0",bustype="socketcan",can_filters=filters)
    
    # load can database TODO remove, no longer used in networking NOT USED
    canDB = cantools.database.load_file("configs/odrive-cansimple.dbc")

    # initialize the controller
    controller = PController(AXIS_ID_LIST, bus_pos_filter, canDB)
    
    # == CAN SETUP == #
    # bus.recv() method appears to have an internal buffer that cannot be cleared or ignored
    # To read current frames being sent, need to register a listener object with a Notifier
    can.Notifier(bus_pos_filter,[controller])
    
    try:
        controller.controller_loop() # loop will block untill node shutdown
    except rospy.ROSInterruptException:
        # always close bus regardless of exception
        print("closing can bus" + bus.channel_info)
        bus_pos_filter.shutdown()