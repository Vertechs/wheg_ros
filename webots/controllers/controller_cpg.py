import time
import numpy as np

# cpg and tweed imports
from wheg_utils.generators.kuramoto_net import GeneratorKuramoto
from wheg_utils import robot_config

# webots imports
try:
    from controller import Robot
    robot = Robot()
except:
    robot = None



