import numpy as np
from numpy import sin,cos,pi,arctan2

from wheg_utils.generators.central_pattern_generator import CPG
from wheg_utils.four_bar_wheg_circ import WhegFourBar
from wheg_utils.robot_config import RobotDefinition

# Implementing Hopf oscillator with variable speed
# model from DOI:10.1109/ROBOT.2008.4543306)

class GeneratorHopfMod(CPG):
    def __init__(self, num_oscillators : int, robot : RobotDefinition):
        #==Constant Parameters==#
        self.N = num_oscillators
        self.freq_const = np.zeros(self.N)
        self.weights_converge = np.ones((2,self.N))
        self.amplitude = np.ones(self.N)
        self.weights_inter = np.ones((self.N,self.N))
        self.bias_inter = np.zeros((self.N,self.N))
        
        self.b_q_off = np.array([[0 ,1 ,2 ,3],
                                 [-1,0 ,1, 1],
                                 [-2,-1 ,0 ,1],
                                 [-1,-2,-1,0]]) * (np.pi/2)
        self.R = self.R_matrix(0) # intermediate term for update math
        self.inter_sum = np.zeros(2)

        #==Dynamic Variables==#
        self.state = np.zeros((2,self.N))
        self.freq = np.zeros(self.N)
        self.dstate = np.zeros((2,self.N))
        self.radius = np.zeros(self.N)

        # feedback term applied to both state variables
        self.feedback = np.zeros(2)

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

    def R_matrix(self,theta):
        return  np.array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])

    def euler_update(self, t_step):
        for i in range(self.N):
            # TODO add frequency shaping math
            self.freq = self.freq_const * 1.0
            self.radius[i] = self.state[0,i]**2 + self.state[1,i]**2

            # coupling term applied based on relative phase offset
            self.inter_sum = np.zeros(2)
            for j in range(self.N):
                self.R = self.R_matrix(self.bias_inter[i,j])
                self.inter_sum += self.weights_inter[i,j] * np.matmul(self.R,self.state[:,j])

            # dx/dt = a*(u-r^2)*x - w*y
            self.dstate[0,i] = (self.weights_converge[0,i] * (self.amplitude[i] - self.radius[i]**2) * self.state[0,i]
                                - self.freq[i] * self.state[1, i] + self.inter_sum[0] + self.feedback[0])
            # dy/dt = a*(u-r^2)*y - w*x
            self.dstate[1, i] = (self.weights_converge[1,i] * (self.amplitude[i] - self.radius[i] ** 2) * self.state[1, i]
                                + self.freq[i] * self.state[0, i] + self.inter_sum[1] + self.feedback[1])

        # apply derivatives by euler method
        self.state += self.dstate * t_step

    def set_state(self, n, state):
        self.state[:, n] = state

    def wheel_output(self):
        # return limit circle radius and current phase
        return np.arctan2(self.state[1,:],self.state[0,:]),self.radius

    def graph_output(self):
        return self.state[1,:]

    def diff_input(self,v,w,h):
        differential = (w * self.wheel_dist / self.wheel_rad)
        self.freq_const[:] = (v * self.n_arc / self.wheel_rad) + self.wheel_dir * differential
