import rospy
import odrive
import configparser
import std_msgs.msg as ros_msg
import odrive.enums as onum

# A single node interfacing with all odrives over USB native interface to
# change control modes and update configuration so as not to overload the CAN bus
# switch node also handles publishing on controller status topic to control 
# the 3 control mode nodes and the trajectory senders
# also publishes general status info (batt voltage, etc) since it is the only 
# node connected through the native interface

# TODO maybe use parameter server, not sure if fast enough
# or master slave model for controller nodes

CONTROL_MODES = ['disable','run','walk','roll','run_vel']

class ControllerSwitch:
    def __init__(self,cfg):
        self.mode = 0
        self.last_mode = 0
        
        self.sn_list = cfg.get("Hardware","odrive serial numbers hex")
        self.sn_list = self.sn_list.replace(' ','').split(',')
        
        # set up ROS nodes and topics
        rospy.init_node("controller_switch")
        self.ctrl_status_pub = rospy.Publisher("switch_status", ros_msg.UInt8MultiArray, queue_size = 2)
        self.drive_status_pub = rospy.Publisher("drv_status", ros_msg.String, queue_size=10)
 
        
        self.ctrl_msg = ros_msg.UInt8MultiArray()
        self.drv_msg = ros_msg.String()
        
        # populate list of odrive fibre objects, will be in whatever order 
        # is listed in config.ini
        self.drives = []
        self.axes = []
        rospy.loginfo("Connecting to odrives...")
        for sn in self.sn_list:
            try:
                drv = odrive.find_any(serial_number = sn, timeout=10)
                self.drives.append(drv)
                self.axes.append(drv.axis0)
                self.axes.append(drv.axis1)
            except TimeoutError:
                print("Couldnt find odrive ", str(sn))
                self.drives.append(None)
                self.axes.append(None)
                self.axes.append(None)
                
        # init clock for sending periodic status messages, per 10s
        self.clock = rospy.Rate(0.1)
        
        # planner at end in case get interrupted during setup
        rospy.Subscriber("planner_mode", ros_msg.String, self.planner_callback)
        
        rospy.loginfo("Setup complete")
    
    ### callback for messages published from path planning
    # will block for quite a while doing setup, may miss messages?
    def planner_callback(self, msg):
        rospy.loginfo(msg)
        
        # update control_switch message, assume message is ros_msg.String matching control mode list
        try:
            new_mode = CONTROL_MODES.index(msg.data)
            self.last_mode = self.mode
            self.mode = new_mode
            self.ctrl_msg.data = [self.mode]
        except ValueError:
            # if not in control mode list, dont update and send a warning
            rospy.logwarn("Invalid control mode, falling back to " + CONTROL_MODES[self.mode])
        
        # broadcast new control status, can bus should be clear after one control cycle. (~80hz)
        self.ctrl_status_pub.publish(self.ctrl_msg)
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
        
        
    # main loop only sends periodic status messages to controller nodes
    def main_loop(self):
        while not rospy.is_shutdown():
            # get battery voltage from first drive
            self.drv_msg.data = "vbus=%.2f"%self.drives[0].vbus_voltage
            # get drive errors and append to msg ros_msg.String
            for drive in self.drives:
                pass
                
            rospy.loginfo(self.drv_msg.data)
                
            # publish status messages
            self.ctrl_status_pub.publish(self.ctrl_msg)
            self.drive_status_pub.publish(self.drv_msg)
            
            self.clock.sleep()
        
        rospy.loginfo("Exiting")
        ### exit logic here
        

    # startup sequence, run every time bring robot up from disabled state
    def startup(self):
        for ax in self.axes:
            # reset encoder estimates so in case controllers lost track
            ax.encoder.set_linear_count(0)
            ax.requested_state = onum.AxisState.CLOSED_LOOP_CONTROL
            
    # shutdown motor, idle all axes, encoders still tracking
    def shutdown(self):
        rospy.loginfo("Disabling motors")
        for ax in self.axes:
            ax.requested_state = onum.AxisState.IDLE

    # switch to running mode
    def start_running(self):
        rospy.loginfo("Switching to running mode")
        for ax in self.axes:
            # set control and input modes
            ax.controller.input_vel = 0.0 #??should preserve probably
            ax.controller.config.control_mode = onum.ControlMode.VELOCITY_CONTROL
            ax.controller.config.input_mode = onum.InputMode.PASSTHROUGH
            
            # set new odrive internal controller gains
            ax.controller.config.vel_gain = 0.04
            ax.controller.config.pos_gain = 20 # not used in vel control
            ax.controller.config.vel_integrator_gain = 0.01
            ax.controller.config.vel_integrator_limit = 100.0
        
        # set CPG configs
        ### here
        
    def start_walking():
        rospy.loginfo("Switching to walking mode")
        for ax in self.axes:
            # hold current position when switching modes
            ax.controller.input_pos = ax.encoder.estimate
            ax.controller.config.control_mode = onum.ControlMode.POSITION_CONTROL
            ax.controller.config.input_mode = onum.InputMode.POS_FILTER
            
            # set new odrive internal controller gains
            ax.controller.config.vel_gain = 0.04
            ax.controller.config.pos_gain = 120 
            ax.controller.config.vel_integrator_gain = 0.01
            ax.controller.config.vel_integrator_limit = 100.0
            
    def start_rolling():
        rospy.loginfo("Switching to rolling mode")
        for ax in self.axes:
            ax.controller.input_vel = 0.0 #??
            ax.controller.config.control_mode = onum.ControlMode.VELOCITY_CONTROL
            ax.controller.input_mode = onum.InputMode.PASSTHROUGH
        
        

if __name__ == "__main__":
    # load config, assuming run from root "/wheg" directory
    
    # TODO Get from ros package path
    #pkg_path = 
    
    cfg = configparser.ConfigParser()
    cfg.read("configs/robot_config.ini")

    try:
        node = ControllerSwitch(cfg)
        node.main_loop()
        
    except rospy.ROSInterruptException:
        pass
    
            