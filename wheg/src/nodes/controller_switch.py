import rospy
import odrive
import can
import configparser
import std_msgs.msg as ros_msg
import odrive.enums as onum
import time
import csv
import numpy as np

from wheg_utils import robot_config

# A single node interfacing with all odrives over USB native interface to
# change control modes and update configuration so as not to overload the CAN bus
# switch node also handles publishing on controller status topic to control 
# the 3 control mode nodes and the trajectory senders
# also publishes general status info (batt voltage, etc) since it is the only 
# node connected through the native interface

# TODO maybe use parameter server, not sure if fast enough
# or master slave model for controller nodes

CONTROL_MODES = ['disable','run','walk','roll','run_rtr','rstart','rstop']

AXIS_ID_LIST = [a for a in range(0x0A,0x12)]
CSV_LENGTH = 2000

class ControllerSwitch:
    def __init__(self,cfg,can_bus,ax_ids):
        self.mode = 0
        self.last_mode = 0
        
        self.can_mode = 0
        
        self.sn_list = cfg.drive_sn
        
        self.can_bus = can_bus
        self.ax_ids = ax_ids
        # heartbeat request frames for each axis
        self.rtr_msgs = [can.Message(arbitration_id = 0x01 | a<<5, dlc = 8, 
                        is_extended_id = False, is_remote_frame = True) for a in ax_ids]
        
        # set up ROS nodes and topics
        rospy.init_node("controller_switch")
        self.ctrl_status_pub = rospy.Publisher("switch_mode", ros_msg.UInt8MultiArray, queue_size = 2)
        #self.drive_status_pub = rospy.Publisher("drv_status", ros_msg.String, queue_size=10)
        
        self.ctrl_msg = ros_msg.UInt8MultiArray()
        self.drv_msg = ros_msg.String()
        
        self.ctrl_msg.data = [0] # start with motors disabled
        self.recording = False
        self.r_msg = 0
        
        # populate list of odrive fibre objects, will be in whatever order 
        # is listed in config.ini
        self.drives = []
        self.axes = []
        rospy.loginfo("Connecting to odrives...")
        for sn in self.sn_list:
            try:
                drv = odrive.find_any(serial_number = sn, timeout=20)
                rospy.loginfo("Found " + str(sn))
                self.drives.append(drv)
                self.axes.append(drv.axis0)
                self.axes.append(drv.axis1)
            except TimeoutError:
                print("Couldnt find odrive ", str(sn))
                self.drives.append(None)
                self.axes.append(None)
                self.axes.append(None)
                
        self.n_ax = len(self.axes)
                
        # init clock for sending periodic status messages, per 10s
        self.clock = rospy.Rate(10)
        self.i_count = 0
        self.r_count = 0 # line count
        self.f_count = 0 # file count
        
        self.r_data = np.zeros((CSV_LENGTH,17))
        
        # planner at end in case get interrupted during setup
        rospy.Subscriber("planner_mode", ros_msg.String, self.planner_callback)
        
        # ros mobile topics for limited remote control
        #rospy.Subscriber("stop_btn", ros_msg.Bool, self.stop_callback)
        #self.stop_btn_val = False
        
        rospy.loginfo("Setup complete")
    
    ### callback for messages published from path planning
    # will block for quite a while doing setup, may miss messages?
    def planner_callback(self, msg):
        # if msg.data == 'rstart':
            # rospy.loginfo('Recording')
            # self.r_count = 0
            # self.recording = True
            # return
        
        # elif msg.data == 'rstop':
            # self.recording = False
            # rospy.loginfo('Writing to csv')
            # with open('data/run_'+str(self.f_count)+'.csv','w') as f:
            #     w = csv.writer(f)
            #     for i in range(CSV_LENGTH):
            #         w.writerow(self.r_data[i,:])
            # self.f_count += 1
            # self.r_count = 0
            
            # return
            
            
        
        # update control_switch message, assume message is ros_msg.String matching control mode list
        try:
            new_mode = CONTROL_MODES.index(msg.data)
            self.last_mode = self.mode
            self.mode = new_mode
            
            if new_mode == 5:
                self.r_msg = 1
            elif new_mode == 6:
                self.r_msg = 0
            elif new_mode == 4:
                self.can_mode = 1
            else:
                self.can_mode = 0
            
        except ValueError:
            # if not in control mode list, dont update and send a warning
            rospy.logwarn("Invalid control mode, falling back to " + CONTROL_MODES[self.mode])
        
        self.ctrl_msg.data = [self.mode,self.can_mode,self.r_msg]
        # Control nodes should always exit with zero velocity?
        # TODO, pass commanded velcoity and phases to other node?
        
        # if switching to any state from disabled state
        if self.mode != 0 and self.last_mode == 0:
            self.startup()
            
        # run startup functions depending on new mode
        if self.mode == 0:
            self.shutdown()
        elif self.mode == 1:
            self.start_running()
        elif self.mode == 2:
            self.start_walking()
        elif self.mode == 3:
            self.start_rolling()
            
        # if self.can_mode == 1:
        #     self.disable_can_cyclic()
        # else:
        #     self.enable_can_cyclic()
            
        # broadcast new control status whenever it is updated
        self.ctrl_status_pub.publish(self.ctrl_msg)
            
    # disable motors if stop button pushed
    def stop_callback(self,msg):
        if msg.data:
            self.shutdown()
    
    # Odrive cyclic message sending is on by default, disable and enable them
    # depending on the controller in use
    def enable_can_cyclic(self):
        if self.mode == 2: # Walking controller requires slower input
            enc_rate = 20
        else:
            enc_rate = 10
        
        for axis in self.axes:
            axis.config.can.heartbeat_rate_ms = 1000
            axis.config.can.encoder_rate_ms = 50 # TODO replace
            
        # Have to send an rtr frame to each drive to reset the cyclic message server
        for msg in self.rtr_msgs[::2]:
            self.can_bus.send(msg)
            
        rospy.loginfo("CAN cyclic messages on")
                
    def disable_can_cyclic(self):
        for axis in self.axes:
            axis.config.can.heartbeat_rate_ms = 0
            axis.config.can.encoder_rate_ms = 0    
        
        rospy.loginfo("CAN cyclic messages off")
        

        
    # main loop only sends periodic status messages to controller nodes
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.i_count % 100 == 0:
                # get battery voltage from first drive
                self.drv_msg.data = "vbus=%.2f"%self.drives[0].vbus_voltage
                # get drive errors and append to msg ros_msg.String
                for i in range(self.n_ax):
                    if self.axes[i].error:
                        rospy.logwarn("Drive %d error, shutting down"%(i//2))
                        odrive.utils.dump_errors(self.drives[i//2])
                        self.shutdown()
                        quit()
                    
                rospy.loginfo(self.drv_msg.data)
                
    
                # publish status messages
                self.ctrl_status_pub.publish(self.ctrl_msg)
                #self.drive_status_pub.publish(self.drv_msg)
                
            if self.recording:
                t0 = time.monotonic_ns()
                self.r_data[self.r_count,0] = time.monotonic()
                for i in range(8):
                    self.r_data[self.r_count,i+1] = self.axes[i].encoder.pos_estimate
                for i in range(8):
                    self.r_data[self.r_count,i+8] = self.axes[i].motor.current_control.Iq_measured
                self.r_count += 1
                if self.r_count > CSV_LENGTH:
                    rospy.logwarn('data overflow')
                    self.r_count = 0
                t1 = time.monotonic_ns()
                print((t1-t0)//1000)
                
                
            self.i_count += 1
            
            self.clock.sleep()
        
        rospy.loginfo("Exiting")
        self.shutdown()
        self.can_bus.shutdown()
        ### exit logic here
        

    # startup sequence, run every time bring robot up from disabled state
    def startup(self):
        for ax in self.axes:
            # reset encoder estimates in case controller nodes lost track
            ax.encoder.set_linear_count(0)
            ax.requested_state = onum.AxisState.CLOSED_LOOP_CONTROL
            
        # # enable cyclic by default
        self.enable_can_cyclic()
            
    # shutdown motor, idle all axes, reset encoders to 0 but still tracking
    def shutdown(self):
        rospy.loginfo("Disabling motors")
        for ax in self.axes:
            ax.requested_state = onum.AxisState.IDLE
            
        for ax in self.axes:
            ax.encoder.set_linear_count(0)

        # publish disabled state to controller nodes
        self.mode = 0
        self.ctrl_msg.data[0] = 0
        self.ctrl_status_pub.publish(self.ctrl_msg)

    # switch to running mode
    def start_running(self):
        rospy.loginfo("Switching to running mode")
        for ax in self.axes:
            # set control and input modes
            ax.controller.input_vel = 0.0 #??should preserve probably
            ax.controller.config.control_mode = onum.ControlMode.VELOCITY_CONTROL
            ax.controller.config.input_mode = onum.InputMode.PASSTHROUGH
            
            # set new odrive internal controller gains
            # TODO figure out correct gains
            ax.controller.config.vel_gain = 0.04
            ax.controller.config.pos_gain = 20 # not used in vel control
            ax.controller.config.vel_integrator_gain = 0.01
            ax.controller.config.vel_integrator_limit = 100.0
            
        
        # set CPG configs
        ### here
        
    def start_walking(self):
        rospy.loginfo("Switching to walking mode")
        for ax in self.axes:
            # odrive sets input_pos to current pos automatically when going to position control
            ax.encoder.set_linear_count(0)
            ax.controller.config.control_mode = onum.ControlMode.POSITION_CONTROL
            ax.controller.config.input_mode = onum.InputMode.POS_FILTER
            ax.controller.config.input_filter_bandwidth = 10 # TODO from parameters
            
            # set new odrive internal controller gains
            ax.controller.config.vel_gain = 0.04
            ax.controller.config.pos_gain = 120 
            ax.controller.config.vel_integrator_gain = 0.01
            ax.controller.config.vel_integrator_limit = 10.0
            
            
    def start_rolling(self):
        rospy.loginfo("Switching to rolling mode")
        for ax in self.axes:
            ax.controller.input_vel = 0.0 # TODO
            ax.controller.config.control_mode = onum.ControlMode.VELOCITY_CONTROL
            ax.controller.config.input_mode = onum.InputMode.PASSTHROUGH
            
            # set new odrive internal controller gains
            ax.controller.config.vel_gain = 0.1
            ax.controller.config.pos_gain = 20 
            ax.controller.config.vel_integrator_gain = 0.0
            ax.controller.config.vel_integrator_limit = 100.0
        
        

if __name__ == "__main__":
    # load config, assuming run from root "/wheg" directory
    
    # cfg = configparser.ConfigParser()
    # cfg.read("configs/robot_config.ini")
    
    # if len(cfg.sections()) < 1:
    #     raise Exception("Config empty, may be running in the wrong directory?")
    cfg = robot_config.get_config_A()
    
    # need a can bus interface to reset odrive cyclic sending
    bus = can.Bus(channel="can0",bustype="socketcan")

    try:
        node = ControllerSwitch(cfg,bus,AXIS_ID_LIST)
        node.main_loop()
        
    except rospy.ROSInterruptException:
        pass
    
            