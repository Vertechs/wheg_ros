import rospy
import odrive
import std_msgs.msg as ros_msg
import time
import numpy as np

# local packages with wheg and cpg classes
from wheg_utils.CPG_FIC import GeneratorFC
from wheg_utils import robot_config

robot = robot_config.get_config_A()

CPG_RATE = 200

# wheel biases for differential drive and ride height calcs
WHEEL_DIR = [1,-1,1,-1]
WHEEL_BIAS_X = [1,-1,1,-1]
WHEEL_BIAS_Y = [-1, -1, 1, 1]

class Generator:
    def __init__(self,n_whl):
        #=== CPG SETUP ===#
        # init fully connected oscillator network with one for each wheel
        self.cpg = GeneratorFC(n_whl)
        self.n_whl = n_whl

        # "walking" gate, all wheel phases one quarter turn offset
        self.cpg.weights = np.ones((4,4)) - np.eye(4)
        self.cpg.biases = np.array([[0 ,2 ,1 ,3],
                                    [-2,0 ,-1,1],
                                    [-1,1 ,0 ,2],
                                    [-1,-1,-2,0]]) * (np.pi/2)
                                    
        self.amp_multiplier = np.ones((self.n_whl))
        
        #=== ROS Setup ===#
        rospy.init_node("central_pattern_generator")
        
        # #TODO balance cpg update rate with sending rate to controllers
        self.cpg_clock = rospy.Rate(CPG_RATE)
        self.send_ratio = 400 # after how many internal updates to send pos command
        
        self.mode_sub = rospy.Publisher("planner_mode", ros_msg.String, self.mode_callback)
        self.vector_sub = rospy.Publisher("planner_vector", ros_msg.Float32MultiArray, self.vector_callback)
        
        self.target_pub = rospy.Publisher("run_pos_cmd", ros_msg.Float32MultiArray, queue_size = 2)
        
        self.target_message = ros_msg.Float32MultiArray(data = [0.0]*n_whl*2)
        self.ext_tar = [0.0]*n_whl
        self.rot_tar = [0.0]*n_whl

        # wheel circumfrances, saved as list in case they are different
        self.wheel_rad = [mod.radius for mod in robot.modules]
        self.wheel_dist = robot.wheel_base_width
        self.wheel_ext_hight = [mod.ext_ratio*mod.radius for mod in robot.modules]


    def mode_callback(self,msg):
        # TODO lock CPG output to true diff drive when in rolling mode
        pass
    
    def vector_callback(self.msg):
        v,w,h,ax,ay = msg.data # x+ vel, z angular vel, ride height and angle
        
        # differential drive math to set oscilator frequency
        for i in range(self.n_whl):
            self.cpg.own_freq[i] = (WHEEL_DIR[i]*v + w*self.wheel_dist) / self.wheel_rad[i]
        
            # oscilator offset tracks desired extension amount
            # sin(x) = x approximation for ride angle
            self.cpg.target_offset[i] = h/self.wheel_rad[i]
            
            # TODO calculate this from IK or look up table
            # Amplitude used to smooth the wheel rolling, i.e. retracting 
            # slightly when a leg is pointed straight down
            self.amp_multiplier = 0.0
        
    def generator_loop(self)
        t_step = 1.0/CPG_RATE # TODO get this from actual time
        
        while not rospy.is_shutdown():
            self.cpg.euler_update(t_step)
            self.smoothing = 
            self.ext_tar = (self.smoothing + self.cpg.off)
            self.rot_tar = self.cpg.phi.tolist()
            self.target_message.data = self.ext_tar + self.rot_tar
            

if __name__=='__main__':
    gen = Generator(4)
    try:
        gen.generator_loop() # loop will block untill node shutdown
    except rospy.ROSInterruptException:
        print("ROS shutdown")
