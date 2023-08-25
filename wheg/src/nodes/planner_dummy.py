import rospy
import odrive
import std_msgs.msg as ros_msg
import time

# node that takes user input to switch controller modes and send simple position
# commands for testing.

CONTROL_MODES = ['disable','run','walk','roll','run_rtr']

class PlannerPassthrough:
    def __init__(self):
        rospy.init_node("planner_pass")
        self.mode = CONTROL_MODES[0]
        self.mode_pub = rospy.Publisher("planner_mode", ros_msg.String, queue_size = 2)
        self.pose_pub = rospy.Publisher("walk_pos_cmd", ros_msg.Float32MultiArray, queue_size = 2)
        self.diff_pub = rospy.Publisher("planner_vector", ros_msg.Float32MultiArray, queue_size = 2)
        self.switch_pub = rospy.Publisher("switch_mode", ros_msg.UInt8MultiArray, queue_size = 2)
        
        self.pose = ros_msg.Float32MultiArray()
        self.pose.data = [0.0]*8
        
        self.diff = ros_msg.Float32MultiArray()
        self.diff.data = [0.0]*5
        
        self.ctrl_msg = ros_msg.UInt8MultiArray()
    
    def main_loop(self):
        while not rospy.is_shutdown():
            msg = input("Enter controller mode or command: ")
            if msg == 'q':
                quit()
            elif len(msg) < 1:
                print("--invalid mode--")
            elif msg in CONTROL_MODES:
                self.mode_pub.publish(msg)
            elif msg[0] == 'p':
                # try getting position arguments
                try:
                    pos = list(map(float,msg[1:].split()))
                except:
                    print("--invalid pos command--")
                    continue
                
                if len(pos) != 8:
                    print("--pos needs 8 values--")
                    continue
                else:
                    self.pose.data = pos
                    self.pose_pub.publish(self.pose)
            elif msg[0] == 'd':
                # try getting differential drive arguments
                try:
                    diff = list(map(float,msg[1:].split()))
                except:
                    print("--invalid pos command--")
                    continue
                
                if len(diff) != 3:
                    print("--diff needs 3 values--")
                    continue
                else:
                    self.diff.data = diff + [0,0]
                    self.diff_pub.publish(self.diff)
            elif msg[0] == 's':
                msgs = msg.split()
                if msgs[1] in CONTROL_MODES:
                    mode = CONTROL_MODES.index(msgs[1])
                    self.ctrl_msg.data = [mode,0]
                    self.mode_pub.publish(msg)
                    self.switch_pub.publish(self.ctrl_msg)
                else:
                    print("invalid switch mode")
                
                
            else:
                print("--invalid mode--")
                
                


if __name__ == "__main__":
    try:
        node = PlannerPassthrough()
        node.main_loop()
        
    except rospy.ROSInterruptException:
        pass
    
            