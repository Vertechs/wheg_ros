import rospy
import odrive
import std_msgs.msg as ros_msg
import time
import numpy as np

# simple differential drive trajectory generator to test controllers

# local packages with wheg and cpg classes
from wheg_utils import robot_config

robot = robot_config.get_config_A()

# wheel biases for differential drive and ride height calcs
WHEEL_DIR = [-1,1,-1,1] # directions handled backwards in walking controller
WHEEL_BIAS_X = [1,-1,1,-1]
WHEEL_BIAS_Y = [-1, -1, 1, 1]

MAX_VEL = 1.0

class Generator:
    def __init__(self,n_whl):
        #=== ROS Setup ===#
        rospy.init_node("diff_generator")
        
        # #TODO balance cpg update rate with sending rate to controllers
        self.clock = rospy.Rate(100)
        self.send_ratio = 10 # after how many internal updates to send pos command
        
        rospy.Subscriber("switch_mode", ros_msg.UInt8MultiArray, self.mode_callback)
        rospy.Subscriber("planner_vector", ros_msg.Float32MultiArray, self.vector_callback)
        
        self.target_pub = rospy.Publisher("walk_pos_cmd", ros_msg.Float32MultiArray, queue_size = 2)
        
        self.enabled = False
        
        #=== Variable setup ===#
        self.n_whl = n_whl
        
        self.target_message = ros_msg.Float32MultiArray(data = [0.0]*n_whl*2)
        self.ext_tar = [0.0]*n_whl
        self.rot_tar = [0.0]*n_whl
        
        self.wheel_vel = [0.0]*n_whl

        # wheel circumfrances, saved as list in case they are different
        self.wheel_rad = [mod.radius for mod in robot.modules]
        self.wheel_dist = robot.wheel_base_width
        self.wheel_ext_height = [mod.ext_rad_ratio*mod.radius for mod in robot.modules]
        
        rospy.loginfo("Diff drive passthrough launched")


    def mode_callback(self,msg):
        # if controllers disabled, reset to zero position and stop updating
        if msg.data[0] == 0:
            if self.enabled:
                rospy.loginfo("Disabled, reset to zero")
                self.rot_tar = [0.0]*self.n_whl
                self.wheel_vel = [0.0]*self.n_whl
                self.enabled = False
        elif msg.data[0] == 2:
            if not self.enabled:
                rospy.loginfo("Updates enabled")
                self.enabled = True
    
    def vector_callback(self,msg):
        v,w,h,ax,ay = msg.data # x+ vel, z angular vel, ride height and angle
        
        # differential drive math to set wheel velocities
        for i in range(self.n_whl):
            # direction applied to angular vel instead of linear vel
            # controller expects +rotation to mean +x in robots frame
            self.wheel_vel[i] = (v + WHEEL_DIR[i]*w*self.wheel_dist/self.wheel_rad[i]) / self.wheel_rad[i]
            
            if self.wheel_vel[i] > MAX_VEL:
                self.wheel_vel[i] = MAX_VEL
                rospy.logwarn("Too high +speed commanded for wheel %d, falling back to %.2f"%(i,MAX_VEL))
            elif self.wheel_vel[i] < -MAX_VEL:
                self.wheel_vel[i] = -MAX_VEL
                rospy.logwarn("Too high -speed commanded for wheel %d, falling back to %.2f"%(i,-MAX_VEL))
        
            # set extension directly in radians
            self.ext_tar[i] = h #TODO add angle
            
            if self.ext_tar[i] > 1.0:
                self.ext_tar[i] = 1.0
                rospy.logwarn("Too high extension commanded for wheel %d, falling back to 1.0"%i)
            elif self.ext_tar[i] < 0.0:
                self.ext_tar[i] = 0.0
                rospy.logwarn("Too low extension commanded for wheel %d, falling back to 0.0"%i)
        
    def generator_loop(self):
        self.t0 = time.monotonic_ns()
        self.t1 = time.monotonic_ns()
        self.iter_counter = 0

        while not rospy.is_shutdown():
            self.t0 = self.t1
            self.t1 = time.monotonic_ns()
            t_step = (self.t1-self.t0)*10e-9
            
            if self.enabled:
                for i in range(self.n_whl):
                    self.rot_tar[i] += self.wheel_vel[i]*t_step
                
            self.target_message.data = self.rot_tar + self.ext_tar
            
            if self.iter_counter >= self.send_ratio:
                self.iter_counter = 0
                self.target_pub.publish(self.target_message)
                # rospy.loginfo([round(a,2) for a in self.target_message.data])
            
            self.iter_counter += 1
            
            self.clock.sleep()
            

if __name__=='__main__':
    gen = Generator(4)
    try:
        gen.generator_loop() # loop will block untill node shutdown
    except rospy.ROSInterruptException:
        print("ROS shutdown")
