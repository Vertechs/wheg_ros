import time
import numpy as np

# cpg and tweed imports
from wheg_utils.generators.kuramoto_net import GeneratorKuramoto
from wheg_utils import robot_config

cfg = robot_config.get_config_webots()

# webots imports
from controller import Robot, Keyboard

# commands to send: [forward vel, angular vel, height, angle x, angle y]
CMD_F20 = [50, 0.0, 65, 0.0, 0.0]
CMD_E20 = [50, 0.0, 100, 0.0, 0.0]
CMD_STOP = [0.0, 0.0, 65, 0.0, 0.0]
CMD_T02 = [0.0, 0.2, 100, 0.0, 0.0]
CMD_TF = [50, -0.03, 100, 0.0, 0.0]

class ControllerCPG:
    def __init__(self,n_whl,seed):
        self.robot_sim = Robot()
        self.keyboard = Keyboard()
        self.cpg = GeneratorKuramoto(n_whl,cfg)
        self.n_whl = n_whl

        # in milliseconds of simulation time
        self.time_step = self.robot_sim.getBasicTimeStep()
        if self.time_step % 1 != 0 :
            print("timstep not int")
            quit()
        else:
            self.time_step = int(self.time_step)
        self.time = 0
        self.command_iter = 0
        
        # define starting gate, all wheel phases one quarter turn offset
        self.cpg.weights = np.ones((4, 4)) - np.eye(4)
        self.cpg.biases = self.cpg.b_q_off #np.zeros((4,4)) #
        self.cpg.gain_off = 1.5
        self.cpg.gain_amp = 1.5

        # send first command to initilize. shift states to avoid unstable eq
        self.cpg.diff_input(0.0, 0.0, self.cpg.wheel_rad)
        self.cpg.perturbation(0, [0.01, 0.01, 0.01])  # TODO get state from feedback

        self.ext_tar = np.zeros(n_whl)
        self.rot_tar = np.zeros(n_whl)
        self.rot_hat = np.zeros(n_whl)
        self.ext_hat = np.zeros(n_whl)
        self.phi_hat = np.zeros(n_whl * 2)
        self.phi_tar = np.zeros(n_whl * 2)

        # load gear ratios (not really needed for current simulation robot)
        self.inner_ratio = [mod.inner_ratio for mod in cfg.modules]
        self.outer_ratio = [mod.outer_ratio for mod in cfg.modules]
        self.ext_ratio = [mod.whl_ratio for mod in cfg.modules]

        # directions depending on motor location and wheel orientation
        self.ext_dir = [-1, -1, -1, -1]  # extension direction for each wheel s.t. expanding is +
        self.rot_dir = [1, 1, 1, 1]  # rotation direction for each wheel s.t. forward is +

        # simulated robot setup
        self.drives = []
        for i in range(n_whl):
            self.drives.append(self.robot_sim.getDevice('mo_0%d' % i))
            self.drives.append(self.robot_sim.getDevice('mi_0%d' % i))

        # keyboard setup for input
        self.keyboard.enable(int(self.time_step))
        self.vector_message = [0,0,0,0,0]

        # random wheel offset
        self.rng = np.random.RandomState(seed)

        alpha = 2*np.pi/5
        # wheel_offset = (self.rng.rand(4)-0.5)*0.3
        # print('offset to:',wheel_offset)

        inner_offset = self.cpg.wheels[0].p_closed
        self.phi_offset = [0.0,inner_offset]*n_whl
        # for i in range(self.n_whl):
        #     self.phi_offset[2*i] = self.phi_offset[2*i] + wheel_offset[i]
        #     self.phi_offset[2*i+1] = self.phi_offset[2*i+1] + wheel_offset[i]

        print('cpg controller started')


    def update_est(self):
        # get from encoder device
        pass

    def control_math(self):
        self.rot_tar, self.ext_tar = self.cpg.wheel_output()
    
        for i in range(self.n_whl):
            # calculate motor phases from gearing ratios
            self.e = (self.ext_tar[i] * self.ext_ratio[i]) * self.ext_dir[i]
            self.p = self.rot_tar[i] * self.rot_dir[i]  
            self.phi_tar[2 * i] = self.outer_ratio[i] * (self.p - 0.0 * self.e)
            self.phi_tar[2 * i + 1] = self.inner_ratio[i] * (self.p + 1.0 * self.e)

    def send_commands(self):
        for i in range(self.n_whl*2):
            self.p = self.phi_tar[i] + self.phi_offset[i]
            self.drives[i].setPosition(self.p)

    def read_commands(self):
        key = self.keyboard.getKey()
        if key == ord('W'):
            self.vector_callback(CMD_F20)
        if key == ord('E'):
            self.vector_callback(CMD_E20)
        if key == ord('S'):
            self.vector_callback(CMD_STOP)
        if key == ord('D'):
            self.vector_callback(CMD_T02)

        # DEFINE TEST PROGRAM
        # Straight line
        # if self.time > 1000 and self.command_iter == 0:
        #     self.command_iter = 1
        #     self.vector_callback(CMD_F20)

        # if self.time > 3000 and self.command_iter == 1:
        #     self.command_iter = 2
        #     self.vector_callback(CMD_E20)

        # if self.time > 18000 and self.command_iter == 2:
        #     self.command_iter = 3
        #     self.vector_callback(CMD_F20)

        # if self.time > 23000 and self.command_iter == 3:
        #     self.command_iter = 4
        #     self.vector_callback(CMD_STOP)

        # DEFINE TEST PROGRAM
        # Turning
        if self.time > 1000 and self.command_iter == 0:
            self.command_iter = 1
            self.vector_callback(CMD_E20)

        if self.time > 3000 and self.command_iter == 1:
            self.command_iter = 2
            self.vector_callback(CMD_TF)

        if self.time > 18000 and self.command_iter == 2:
            self.command_iter = 3
            self.vector_callback(CMD_F20)

        if self.time > 23000 and self.command_iter == 3:
            self.command_iter = 4
            self.vector_callback(CMD_STOP)


    def vector_callback(self,msg):
        v,w,h,ax,ay = msg
        self.cpg.diff_input(v,w,h)

    def control_loop(self):

        while self.robot_sim.step(self.time_step) != -1:
            
            # get estimates from encoders
            self.update_est()
            
            # step CPG dynamics forward
            self.cpg.euler_update(self.time_step / 1000)
            
            # calculate motor phase targets
            self.control_math()
            
            # send motor position commands
            self.send_commands()
            
            # get input from keyboard
            self.read_commands()

            self.time += self.time_step
            
            #print(self.phi_tar)
            
        print('Shutting down')

    def zero_pos(self):
        # generate random offset for each wheel
        rand_offset = (self.rng.rand(4)-0.5)*0.3
        print("random offset to: ",rand_offset)

        # set cpg states
        self.cpg.set_phase(rand_offset)

        # bring wheels to random zeros
        while self.robot_sim.step(self.time_step) != -1:
            # set all motors to zero position
            for i in range(self.n_whl*2):
                pos = rand_offset[i//2] + self.phi_offset[i]
                self.drives[i].setPosition(pos)

            self.time += self.time_step

            # exit after 1 second
            if self.time > 1000:
                return

def check_setting():
    with open('../../data/settings.txt','r') as f:
        setting = f.read().split(' ')
        si = []
        for s in setting:
            si.append(int(s))
        return(si)

run_n,test_n = check_setting()

if __name__ == '__main__':

    ctrl = ControllerCPG(4,run_n+12)
    
    print('zeroing')
    ctrl.zero_pos()

    print('starting control loop')
    ctrl.control_loop()



