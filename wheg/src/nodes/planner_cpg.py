import rospy
import odrive
import std_msgs.msg as ros_msg
import time
import numpy as np

# local packages with wheg and cpg classes
import wheg_utils.generators.kuramoto_net
from wheg_utils import robot_config

robot = robot_config.get_config_A()

CPG_RATE = 50

# wheel biases for differential drive and ride height calcs
WHEEL_DIR = [1,-1,1,-1]
WHEEL_EXT_DIR = []
WHEEL_BIAS_X = [1,-1,1,-1]
WHEEL_BIAS_Y = [-1, -1, 1, 1]

class Generator:
    def __init__(self,n_whl):
        #=== CPG SETUP ===#
        # init fully connected oscillator network with one for each wheel
        self.cpg = wheg_utils.generators.kuramoto_net.GeneratorKuramoto(n_whl,robot)
        self.n_whl = n_whl

        # "walking" gate, all wheel phases one quarter turn offset
        self.cpg.weights = np.ones((4, 4)) - np.eye(4)
        self.cpg.biases = self.cpg.b_q_off
        
        self.cpg.target_amps = np.array([0.5, 0.5, 0.5, 0.5])
        self.cpg.target_offs = np.zeros(4)
        self.cpg.diff_input(1.0,0.0,self.cpg.wheel_rad)
        #print(self.cpg.target_offs,self.cpg.target_amp_norm)
        
        #=== ROS Setup ===#
        rospy.init_node("central_pattern_generator")
        self.enabled = False
        
        # #TODO balance cpg update rate with sending rate to controllers
        self.cpg_clock = rospy.Rate(CPG_RATE)
        self.send_ratio = 2 # after how many internal updates to send pos command
        
        self.mode_sub = rospy.Subscriber("switch_mode", ros_msg.UInt8MultiArray, self.mode_callback)
        self.vector_sub = rospy.Subscriber("planner_vector", ros_msg.Float32MultiArray, self.vector_callback)
        
        self.target_pub = rospy.Publisher("walk_pos_cmd", ros_msg.Float32MultiArray, queue_size = 2)
        
        self.target_message = ros_msg.Float32MultiArray(data = [0.0]*n_whl*2)
        self.ext_tar = [0.0]*n_whl
        self.rot_tar = [0.0]*n_whl

        # NOT USED, handled in generator object
        # # wheel circumfrances, saved as list in case they are different
        # self.wheel_rad = [mod.radius for mod in robot.modules]
        # self.wheel_dist = robot.wheel_base_width
        # self.wheel_ext_hight = [mod.ext_ratio*mod.radius for mod in robot.modules]
        
        rospy.loginfo("CPG started")


    def mode_callback(self,msg):
        # if controllers disabled, reset to zero position and stop updating
        if msg.data[0] != 2:
            if self.enabled:
                rospy.loginfo("Disabled, reset to zero")
                self.cpg.diff_input(0.1,0.0,self.cpg.wheel_rad)
                self.cpg.reset_oscillators()
                self.enabled = False
        elif msg.data[0] == 2:
            if not self.enabled:
                rospy.loginfo("Updates enabled")
                self.enabled = True
    
    def vector_callback(self,msg):
        v,w,h,ax,ay = msg.data # x+ vel, z angular vel, ride height and angle
        # call pseudo-differential drive controller in CPG
        self.cpg.diff_input(v,w,h)
        
        
    def generator_loop(self):
        t_step = 1.0/CPG_RATE # TODO get this from actual time
        self.iter_counter = 0
        
        while not rospy.is_shutdown():
            t1 = time.monotonic_ns()
            if self.enabled:
                self.cpg.euler_update(t_step)
                self.rot_tar,self.ext_tar = self.cpg.wheel_output()
                self.target_message.data = self.rot_tar + self.ext_tar
            
                if self.iter_counter >= self.send_ratio:
                    self.iter_counter = 0
                    self.target_pub.publish(self.target_message)
                    t2 = time.monotonic_ns()
                    rospy.loginfo([round(a,2) for a in self.target_message.data])
                #rospy.loginfo((t2-t1)//1000)
            
            self.iter_counter += 1
            
            self.cpg_clock.sleep()
            

if __name__=='__main__':
    gen = Generator(4)
    try:
        gen.generator_loop() # loop will block untill node shutdown
    except rospy.ROSInterruptException:
        print("ROS shutdown")
