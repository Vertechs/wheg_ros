import rospy
import odrive
import std_msgs.msg as ros_msg
import time
import numpy as np

# local packages with wheg and cpg classes
import wheg_utils.generators.van_der_pol_net
from wheg_utils import robot_config

robot = robot_config.get_config_A()

CPG_RATE = 100

# wheel biases for differential drive and ride height calcs
WHEEL_DIR = [1,-1,1,-1]
WHEEL_EXT_DIR = []
WHEEL_BIAS_X = [1,-1,1,-1]
WHEEL_BIAS_Y = [-1, -1, 1, 1]

class Generator:
    def __init__(self,n_whl):
        #=== CPG SETUP ===#
        # init fully connected oscillator network with one for each wheel
        self.cpg = wheg_utils.generators.van_der_pol_net.GeneratorVdpNet(4,robot)
        self.n_whl = n_whl

        # "walking" gate, all wheel phases one quarter turn offset
        self.cpg.a[:] = np.ones(4) * 1.5
        self.cpg.p_2[:] = np.ones(4) * 2.0
        self.cpg.w_2[:] = np.ones(4) * 4.0
        
        self.cpg.weights[:] = self.cpg.w_walk * 0.2
        
        for i in range(4):
            self.cpg.set_state(i,0.01*i,0.01*i)
        
        #=== ROS Setup ===#
        rospy.init_node("central_pattern_generator")
        self.enabled = False
        
        # #TODO balance cpg update rate with sending rate to controllers
        self.cpg_clock = rospy.Rate(CPG_RATE)
        self.send_ratio = 1 # after how many internal updates to send pos command
        self.print_ratio = 100
        
        self.mode_sub = rospy.Subscriber("switch_mode", ros_msg.UInt8MultiArray, self.mode_callback)
        self.vector_sub = rospy.Subscriber("planner_vector", ros_msg.Float32MultiArray, self.vector_callback)
        #self.estimate_sub = rospy.Subscriber("walk_pos_est", ros_msg.Float32MultiArray, self.estimate_callback)
        
        self.target_pub = rospy.Publisher("walk_pos_cmd", ros_msg.Float32MultiArray, queue_size = 2)
        
        self.target_message = ros_msg.Float32MultiArray(data = [0.0]*n_whl*2)
        self.ext_tar = [0.0]*n_whl
        self.rot_tar = [0.0]*n_whl
        self.rot_hat = [0.0]*n_whl
        self.ext_hat = [0.0]*n_whl

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
                self.cpg.diff_input(10.0,0.0,self.cpg.wheel_rad)
                self.cpg.reset_oscillators()
                self.iter_counter = 0
                self.enabled = False
        elif msg.data[0] == 2:
            if not self.enabled:
                rospy.loginfo("Updates enabled")
                self.enabled = True
    
    
    def vector_callback(self,msg):
        v,w,h,ax,ay = msg.data # x+ vel, z angular vel, ride height and angle
        # call pseudo-differential drive controller in CPG
        self.cpg.diff_input(v,w,h)

    def estimate_callback(self,msg):
        self.rot_hat[:] - msg.data[0:4]
        self.ext_hat[:] - msg.data[0:4]
        
        
    def generator_loop(self):
        t_step = 1.0/CPG_RATE # TODO get this from actual time
        self.iter_counter = 0
        
        while not rospy.is_shutdown():
            t0 = time.monotonic_ns()
            if self.enabled:
                # update oscillator states from estimates
                #self.cpg.wheel_input(self.rot_hat,self.ext_hat)
                
                # step oscillators forward and take output
                self.cpg.euler_update(t_step)
                self.rot_tar,self.ext_tar = self.cpg.wheel_output()
                self.target_message.data = self.rot_tar + self.ext_tar
                t1 = time.monotonic_ns()
                
                # send position commands
                if self.iter_counter % self.send_ratio == 0:
                    self.target_pub.publish(self.target_message)
                    
                t2 = time.monotonic_ns()
                
                # print loop time and current commands
                if self.iter_counter % self.print_ratio == 0:
                    rospy.loginfo([(t1-t0)//1000,(t2-t1)//1000]
                                    + [round(a,2) for a in self.target_message.data])
            
            self.iter_counter += 1
            
            self.cpg_clock.sleep()
            

if __name__=='__main__':
    gen = Generator(4)
    try:
        gen.generator_loop() # loop will block untill node shutdown
    except rospy.ROSInterruptException:
        print("ROS shutdown")
