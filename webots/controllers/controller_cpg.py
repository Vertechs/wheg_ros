import time
import numpy as np

# cpg and tweed imports
from wheg_utils.generators.kuramoto_net import GeneratorKuramoto
from wheg_utils import robot_config

cfg = robot_config.get_config_webots()

# webots imports
from controller import Robot, Keyboard

# commands to send: [forward vel, angular vel, height, angle x, angle y]
CMD_F20 = [200, 0.0, 65]
CMD_E20 = [200, 0.0, 120]
CMD_STOP = [0.0, 0.0, 65]

class ControllerCPG:
    def __init__(self,n_whl,robot_obj,keyboard_obj):
        self.robot_sim = robot_obj
        self.keyboard = keyboard_obj
        self.cpg = GeneratorKuramoto(n_whl,cfg)
        self.n_whl = n_whl

        # in milliseconds of simulation time
        self.time_step = self.robot_sim.getBasicTimestep()

        # define starting gate, all wheel phases one quarter turn offset
        self.cpg.weights = np.ones((4, 4)) - np.eye(4)
        self.cpg.biases = self.cpg.b_q_off
        self.cpg.gain_off = 1.5
        self.cpg.gain_amp = 1.5

        # send first command to initilize. shift states to avoid unstable eq
        self.cpg.diff_input(10, 0.0, self.cpg.wheel_rad)
        self.cpg.perturbation(0, [0.01, 0.01, 0.01])  # TODO get state from feedback

        self.ext_tar = np.zeros(n_whl)
        self.rot_tar = np.zeros(n_whl)
        self.rot_hat = np.zeros(n_whl)
        self.ext_hat = np.zeros(n_whl)
        self.phi_hat = np.zeros(n_whl * 2)
        self.phi_tar = np.zeros(n_whl * 2)

        # load gear ratios (not really needed for current simulation robot)
        self.inner_ratio = [mod.inner_ratio for mod in robot.modules]
        self.outer_ratio = [mod.outer_ratio for mod in robot.modules]
        self.ext_ratio = [mod.whl_ratio for mod in robot.modules]

        # directions depending on motor location and wheel orientation
        self.ext_dir = [-1, -1, -1, -1]  # extension direction for each wheel s.t. expanding is +
        self.rot_dir = [1, 1, 1, 1]  # rotation direction for each wheel s.t. forward is +

        # simulated robot setup
        self.drives = []
        for i in range(n_whl):
            self.drives.append(self.robot_sim.getDevice('mO_0%d' % i))
            self.drives.append(self.robot_sim.getDevice('mI_0%d' % i))

        # keyboard setup for input
        self.keyboard.enable(self.time_step)
        self.vector_message = [0,0,0]

        print('cpg controller started')


    def update_est(self):
        # get from encoder device
        pass

    def control_math(self):
        for i in range(self.n_whl):
            # calculate motor phases from gearing ratios
            self.e = (self.ext_tar[i] * self.ext_ratio[i]) * self.ext_dir[i]
            self.p = self.rot_tar[i] * self.rot_dir[i]  # TODO check ratios??
            self.phi_tar[2 * i] = self.outer_ratio[i] * (self.p - 0.0 * self.e) * CIRC
            self.phi_tar[2 * i + 1] = self.inner_ratio[i] * (self.p + 1.0 * self.e) * CIRC

    def send_commands(self):
        for i in range(self.n_whl*2):
            self.drives[i].setPosition(self.phi_tar[i])

    def read_keyboard(self):
        key = keyboard.getKey()
        if key == ord('W'):
            self.vector_callback(CMD_F20)
        if key == ord('E'):
            self.vector_callback(CMD_E20)
        if key == ord('S'):
            self.vector_callback(CMD_STOP)

    def vector_callback(self,msg):
        v,w,h,ax,ay = msg
        self.cpg.diff_input(v,w,h)

    def control_loop(self):

        while self.robot_sim.step(self.time_step) != -1:
            self.update_est()
            self.cpg.euler_update(self.time_step / 1000)
            self.control_math()
            self.send_commands()
            self.read_keyboard()




if __name__ == '__main__':
    robot = Robot()
    keyboard = Keyboard()
    ctrl = ControllerCPG(4,robot,keyboard)

    try:
        ctrl.control_loop()
    except:
        print('Shutdown')




