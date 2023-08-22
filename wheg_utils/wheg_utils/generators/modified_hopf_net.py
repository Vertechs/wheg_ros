import numpy as np

from wheg_utils.generators.central_pattern_generator import CPG
from wheg_utils.four_bar_wheg_circ import WhegFourBar
from wheg_utils.robot_config import RobotDefinition

# Implementing Hopf oscillator with variable speed
# model from DOI:10.1109/ROBOT.2008.4543306)

class GeneratorHopfMod(CPG):
    def __init__(self, num_oscillators : int, robot : RobotDefinition):
        #==Constant Parameters==#
        self.N = num_oscillators
        self.freq_const = np.zeros((1,self.N))
        self.weights_converge = np.ones((2,self.N))
        self.amplitude = np.ones((1,self.N))
        self.weights_inter = np.ones((self.N,self.N))

        #==Dynamic Variables==#
        self.state = np.zeros((2,self.N))
        self.freq = np.zeros((1,self.N))
        self.dstate = np.zeros((2,self.N))
        self.radius = np.zeros((1,self.N))

        # input applied only to the second state variable (y)
        self.input = 0.0

        # pseudo-differential drive parameters
        # need wheg objects for each oscillator to get basic parameters and do some inverse kinematics
        self.wheels = []
        self.n_arc = []
        for i in range(self.N):
            self.wheels.append(WhegFourBar(robot.modules[i].four_bar.get_parameter_list()))
        self.wheel_dist = robot.wheel_base_width
        self.n_arc = robot.modules[0].n_arc
        self.wheel_rad = robot.modules[0].radius
        self.wheel_dir = np.array([-1,1,-1,1]) # oscillator phases move positive, output must be flipped for left wheels
        self.height = 0.0


    def euler_update(self, t_step):
        for i in range(self.N):
            # TODO add frequency shaping math
            self.freq = self.freq_const * 1.0
            self.radius[0,i] = np.linalg.norm(self.state[:,i])

            # coupling term is applied only on the second state variable (y)
            inter_sum = 0
            for j in range(self.N):
                inter_sum += self.weights_inter[i,j] * self.state[1,j]

            # dx/dt = a*(u-r^2)*x - w*y
            self.dstate[0,i] = (self.weights_converge[0,i] * (self.amplitude[0,i] - self.radius[0,i]**2) * self.state[0,i]
                                - self.freq[0,i] * self.state[1,i])
            # dy/dt = a*(u-r^2)*y - w*x
            self.dstate[1, i] = (self.weights_converge[1,i] * (self.amplitude[0,i] - self.radius[0, i] ** 2) * self.state[1, i]
                                 + self.freq[0,i] * self.state[0, i] + inter_sum + self.input)

        # apply derivatives by euler method
        self.state += self.dstate * t_step

    def set_state(self, n, state):
        self.state[:, n] = state

    def phase_output(self):
        # return limit circle radius and current phase
        return np.arctan2(self.state[1,:],self.state[0,:])

    def graph_output(self):
        return self.state[0,:]

    def diff_input(self,v,w,h):
        differential = (w * self.wheel_dist / self.wheel_rad)
        self.freq_const[:] = (v * self.n_arc / self.wheel_rad) + self.wheel_dir * differential
