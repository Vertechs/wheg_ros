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
        self.w_2 = np.ones(self.N)
        self.p_2 = np.ones(self.N)
        self.a = np.ones(self.N)

        # weights should be small compared to amplitude
        self.weights = (np.ones((self.N,self.N)) - np.eye(self.N)) * -0.2
        self.w_trot = np.array([[0,-1,1,-1],
                               [-1,0,-1,1],
                               [-1,1,0,-1],
                               [1,-1,-1,0]]) * 0.2
        self.w_walk = np.array([[0, -1, -1, -1],
                                [-1, 0, -1, -1],
                                [-1, -1, 0, -1],
                                [-1, -1, -1, 0]]) * 0.2

        # Dynamic variables
        self.x = np.zeros(self.N)
        self.y = np.zeros(self.N)
        self.dx = np.zeros(self.N)
        self.dy = np.zeros(self.N)

        # intermediate variables
        self.x_A = 0.0
        self.radius = np.zeros(self.N)
        self.phase = np.zeros(self.N)
        self.ext_amp = np.zeros(self.N)
        self.ext_off = np.zeros(self.N)

        # control variables
        self.forcing = np.zeros(self.N)
        self.time = 0.0 # track time for forcing signal
        self.frq =  0.25
        self.osc_dir = [-1,-1,-1,-1] # osc naturally moves cw, want ccw
        self.f_amp = 0.1

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
        self.wheel_dir = np.array([-1, 1, -1, 1])  # oscillator phases move positive, output must be flipped for left wheels
        self.height = 0.0
        self.random_state = np.random.RandomState(111)

    def euler_update(self,t_step):
        self.time += t_step
        for i in range(self.N):
            self.forcing[:] = 0.0 #sin(self.frq*self.time)*self.f_amp

        for i in range(self.N):
            self.x_A = self.x[i]
            for j in range(self.N):
                self.x_A += self.weights[i,j] * self.x[j]

            self.dx[i] = self.osc_dir[i] * self.y[i]
            self.dy[i] = self.osc_dir[i] * (self.a[i] * (self.p_2[i] - self.x_A**2) * self.dx[i]
                          - (self.w_2[i] * self.x_A) + self.forcing[i])


        self.y_last[:] = self.y
        self.x += self.dx*t_step
        self.y += self.dy*t_step
        self.set_phase_offset()

        self.radius = np.sqrt(np.power(self.x,2) + np.power(self.y,2))
        self.phase = np.arctan2(self.y,self.x) + self.phase_off

    def set_state(self,n,x,y):
        self.x[n] = x
        self.y[n] = y

    def state_output(self,n):
        return np.vstack([self.x[n],self.y[n]])

    def wheel_output(self):
        # normalize x output and use to control extension through the swing
        ext = self.ext_off - self.ext_amp * self.x / (self.p_2)
        rot = self.phase * 2.0 / self.n_arc
        return rot.tolist(),[min(1.11,max(0.0,p)) for p in ext]

    def graph_output(self):
        return self.x

    def perturbation(self, n, sigmas):
        for i in range(4):
            self.set_state(i, sigmas[i] * i, -sigmas[i] * i)

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
        # get phase difference from requested ride height, only calculate if height changes
        # *** this assumes wheels are all the same to lower computation time ***
        if h != self.height:
            self.height = h
            # set desired radius/offset as the extension one half step forward
            # set oscillation amplitude as the difference between half step and bottom dead center
            # phase difference sent to control is relative to completely closed position
            ph, pb = self.wheels[0].calc_phase_diffs(h)

            # print('ph diffs : ', ph, pb)
            self.ext_amp[:] = abs(ph - pb) * 0.5  # oscillation amplitudes
            self.ext_off[:] = (self.wheels[0].p_closed - ph)  # desired radius
        print(self.ext_amp,self.ext_off)
