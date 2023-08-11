import can
import cantools
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
from geometry_msgs.msg import Twist
import struct

## implementing velocity based differential drive controller
## using instead of extending the running controller when
## wheel position is not needed in the low level control loop
## will free up CAN bus and ROS network bandwidth
## take input as default geometry twist message

## TODO NOTES ##aa

AXIS_STATE_IDLE = bytearray([1,0,0,0,0,0,0,0])
AXIS_STATE_CLOSED = bytearray([8,0,0,0,0,0,0,0])
CI_VEL_MODE = bytearray([2,0,0,0,1,0,0,0])

# populate axis IDs, 10 to 18 by default
AXIS_ID_LIST = [a for a in range(0xA,0x12)]

# TODO should load from config
WHEEL_CIRC = 0.07 * 2 * 3.1415 # in meters, angular pos measured in revolutions
WHEEL_DIST = 0.19 # horizontal distance between wheels in meters

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
        
        # target velocity array, one for each wheel
        self.v_tar = 0.0
        self.w_tar = 0.0
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("roll_controller")
        self.clock = rospy.Rate(100)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_status", UInt8MultiArray, self.switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("roll_vel_cmd", Twist, self.vel_callback)
    
    
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running , 2 Walking, 3 Rolling (this)
        if msg.data[0] == 3:
            if not self.enabled:
                rospy.loginfo("Enabled")
                self.enabled = True
        else:
            if self.enabled:
                self.enabled = False
                rospy.loginfo("Disabled")
            
    def vel_callback(self, msg):
        self.v_tar = msg.linear.x
        self.w_tar = msg.angular.z
    
    def controller_loop(self):
        
        # initialize arrays before entering control loop (not sure if actually faster..)
        phi_hat = np.array([float("NaN")]*self.n_ax, dtype=float)
        rx_bytes = [bytearray([0,0,0,0])]*self.n_ax
        rx_id = [int(0)]*self.n_ax
        vel_cmd = 0.0
        trq_cmd = 0.0
        
        # initialize lists of CAN message objects for sending, only set data in the control loop
        vel_msg = [can.Message(arbitration_id = 0x0D | a<<5, dlc = 8, 
                        is_extended_id = False) for a in self.axIDs]
        
        while not rospy.is_shutdown():
            if self.enabled:
                t1 = time.monotonic_ns()
                self.control_iteration(phi_hat,rx_bytes,rx_id,vel_cmd,trq_cmd,vel_msg)
                t2 = time.monotonic_ns()
                rospy.loginfo("Loop took %d"%( (t2-t1) // 1000) )
            # must sleep some for subscriber functions to be called
            self.clock.sleep()
            
        # exit cleanup, shutdown CAN bus
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
        
    def control_iteration(self,phi_hat,rx_bytes,rx_id,vel_cmd,trq_cmd,vel_msg):
        
        
        ## PASS SHUTDOWN COMMAND

        ### may not need positional data
        # # get data from CAN
        # for i in range(n_ax):
        #     msg = self.bus.recv()
        #     rx_bytes[i] = msg.data[0:4]
        #     rx_id[i] = msg.arbitration_id
        
        # # load estimates into float array, assuming axis IDs are in order
        # for i in range(n_ax):
        #     d = (rx_id[i] >> 5) - ax_id_offset
        #     phi_hat[d] = struct.unpack('f',rx_bytes[i])[0]

        # calculate controller response
        # TODO
        # may need velocity integrator to run here
        # need some positional control to maintain collapsed wheel
        
        for i in range(self.n_ax):
            # differential drive math
            # i//2 is odd if on a left wheel ==> clockwise = pos linear
            if i//2%2 == 0:
                vel_cmd = (-self.v_tar + self.w_tar*WHEEL_DIST) / WHEEL_CIRC
            else:
                vel_cmd = (self.v_tar + self.w_tar*WHEEL_DIST) / WHEEL_CIRC
                
            vel_cmd = min(2, max(vel_cmd, -2))
                
            trq_cmd = 0.2
            vel_msg[i].data = struct.pack('f',vel_cmd)+struct.pack('f',trq_cmd)

        # send messages as fast as possible, will lose arbitration to encoder frames
        for msg in vel_msg:
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