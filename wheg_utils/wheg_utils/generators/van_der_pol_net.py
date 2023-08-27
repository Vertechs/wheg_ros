import numpy as np
from numpy import sin,cos,pi,arctan2

from wheg_utils.generators.central_pattern_generator import CPG
from wheg_utils.four_bar_wheg_circ import WhegFourBar
from wheg_utils.robot_config import RobotDefinition


class GeneratorVdpNet(CPG):
    def __init__(self,num_oscillators,robot):
        # Static parameters
        self.N = num_oscillators
        self.para = np.ones((3,self.N)) # a, p^2, w^2

        # weights should be small compared to amplitude
        self.weights = (np.ones((self.N,self.N)) - np.eye(self.N)) * -0.2
        self.w_full = (np.ones((self.N,self.N)) - np.eye(self.N)) * -0.2
        self.w_half = (np.array([]))

        # Dynamic variables
        self.x = np.zeros(self.N)
        self.y = np.zeros(self.N)
        self.dx = np.zeros(self.N)
        self.dy = np.zeros(self.N)

        self.radius = np.zeros(self.N)
        self.phase = np.zeros(self.N)

        self.time = 0.0
        self.frq = np.ones(self.N) * 1.0
        self.osc_dir = [1,1,1,1]

        # phase
        self.phase_off = np.zeros(self.N)
        self.y_last = np.zeros(self.N)

        # pseudo-differential drive parameters
        # need wheg objects for each oscillator to get basic parameters and do some inverse kinematics
        self.wheels = []
        self.n_arc = []
        for i in range(self.N):
            self.wheels.append(WhegFourBar(robot.modules[i].four_bar.get_parameter_list()))
        self.wheel_dist = robot.wheel_base_width
        self.n_arc = robot.modules[0].n_arc
        self.wheel_rad = robot.modules[0].radius
        self.wheel_dir = np.array(
            [-1, 1, -1, 1])  # oscillator phases move positive, output must be flipped for left wheels
        self.height = 0.0
        self.random_state = np.random.RandomState(111)

    def euler_update(self,t_step):
        for i in range(self.N):
            inter_sum = self.x[i]
            for j in range(self.N):
                inter_sum += self.weights[i,j] * self.x[j]
            forcing = 0.0

            self.dx[i] = self.osc_dir[i] * self.y[i]
            self.dy[i] = self.osc_dir[i] * (self.para[0,i] * (self.para[1,i] - inter_sum**2) * self.dx[i]
                          - self.para[2,i] * inter_sum + forcing)


        self.y_last[:] = self.y
        self.x += self.dx*t_step
        self.y += self.dy*t_step
        self.set_phase_offset()

        self.radius = np.sqrt(np.power(self.x,2)+np.power(self.y,2))
        self.phase = np.arctan2(self.y,self.x) + self.phase_off

    def set_state(self,n,x,y):
        self.x[n] = x
        self.y[n] = y

    def state_output(self,n):
        return np.vstack([self.x[n],self.y[n]])

    def wheel_output(self):
        # return x component and current phase
        return self.phase.tolist(),[max(0.0,p) for p in self.x]

    def graph_output(self):
        return self.x

    def set_phase_offset(self):
    # run anytime states are updated to track quadrant changes for smooth arctan
        for i in range(self.N):
            if self.x[i] < 0:
                if self.y[i] < 0.0 and self.y_last[i] > 0.0:
                    self.phase_off[i] += 2 * pi
                if self.y[i] > 0.0 and self.y_last[i] < 0.0:
                    self.phase_off[i] -= 2 * pi
    def diff_input(self,v : float, w : float, h : float):
        if w > 0:
            self.osc_dir = [1,-1,1,-1]
        pass
