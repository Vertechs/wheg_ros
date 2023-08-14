import rospy
import odrive
import configparser
import std_msgs.msg as ros_msg
import odrive.enums as onum
import time


CONTROL_MODES = ['disable','run','walk','roll','run_vel']

class PlannerPassthrough:
    def __init__(self):
        rospy.init_node("planner")
        self.mode = CONTROL_MODES[0]
        self.mode_pub = rospy.Publisher("planner_mode", ros_msg.String, queue_size = 2)
        self.pose_pub = rospy.Publisher("walk_pos_cmd", ros_msg.Float32MultiArray, queue_size = 2)
        
        self.pose = ros_msg.Float32MultiArray()
        self.pose.data = [0.0]*8
    
    def main_loop(self):
        while not rospy.is_shutdown():
            msg = input("Enter controller mode or command: ")
            if msg == 'q':
                quit()
            elif msg in CONTROL_MODES:
                self.mode_pub.publish(msg)
            elif msg[0] == 'p':
                # try getting position arguments TODO not needed for working controller
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
                
            else:
                print("--invalid mode--")
                
                


if __name__ == "__main__":
    try:
        node = PlannerPassthrough()
        node.main_loop()
        
    except rospy.ROSInterruptException:
        pass
    
            