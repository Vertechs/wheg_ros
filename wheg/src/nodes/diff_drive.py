import can
import cantools
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
import struct

## implementing velocity based differential drive controller
## using instead of extending the running controller when
## wheel position is not needed in the low level control loop
## will free up CAN bus and ROS network bandwidth
## take input as default geometry twist message

## TODO NOTES ##

AXIS_STATE_IDLE = bytearray([1,0,0,0,0,0,0,0])
AXIS_STATE_CLOSED = bytearray([8,0,0,0,0,0,0,0])
CI_VEL_MODE = bytearray([2,0,0,0,1,0,0,0])

# populate axis IDs, 10 to 18 by default
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
        
        # target velocity array, one for each wheel
        self.targets = [0.0]*(n_ax//2)
        self.gains = [1.0]*(n_ax//2)
        
        # initialize lists of CAN message objects for sending, only set data in the control loop
        self.vel_msg = [can.Message(arbitration_id = 0x0D | a<<5, dlc = 8, 
                        is_extended_id = False) for a in axIDs]
    
        #====ROS Setup====#
        # init ros node, not anonymous: should never have 2 instances using same bus
        rospy.init_node("roll_controller")
        self.clock = rospy.Rate(100)
        
        # init subscribers
        self.switch_subscriber = rospy.Subscriber("switch_status", UInt8MultiArray, switch_callback)
        self.pos_command_subscriber = rospy.Subscriber("roll_vel_cmd", Float32MultiArray, vel_callback)
    
    
    def switch_callback(self, msg):
        # control mode is first int of status array
        # 0 Disabled, 1 Running , 2 Walking, 3 Rolling (this)
        if msg.data[0] == 3:
            self.enabled = True
        else:
            self.enabled = False
            
    def vel_callback(self, msg):
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

        # get data from CAN
        for i in range(n_ax):
            msg = self.bus.recv()
            rx_bytes[i] = msg.data[0:4]
            rx_id[i] = msg.arbitration_id
        
        # load estimates into float array, assuming axis IDs are in order
        for i in range(n_ax):
            d = (rx_id[i] >> 5) - ax_id_offset
            phi_hat[d] = struct.unpack('f',rx_bytes[i])[0]

        # calculate controller response
        # TODO
        # may need velocity integrator to run here
        # need some positional control to maintain collapsed wheel
        
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
    
    # initialize the controller
    controller = PController(AXIS_ID_LIST, bus_pos_filter, canDB)
    
    try:
        controller.controller_loop() # loop will block untill node shutdown
    # always close bus regardless of exception
    except rospy.ROSInterruptException:
        print("closing can bus" + bus.channel_info)
        bus_pos_filter.shutdown()
    except:
        print("non interrupt error, closing can bus")
        bus_pos_filter.shutdown()