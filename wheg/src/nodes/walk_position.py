import can
import rospy
import cantools
import time
import numpy as np
import struct
from std_msgs import Float32MultiArray, UInt8MultiArray
from wheg_utils.whegs_4bar import WhegFourBar # local package, install as editable