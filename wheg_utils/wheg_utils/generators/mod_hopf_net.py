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
        self.amplitudes = np.ones(self.N)
        self.weights_inter = np.ones((self.N,self.N))
        self.bias_inter = np.zeros((self.N,self.N))
        self.freq_off = np.ones(self.N) * 0.1
        self.ext_gains = np.ones(self.N) * 0.1
        self.ext_amp = np.zeros(self.N)

        # phase bias arrays for starting and turning
        self.b_q_off = np.array([[0 ,1 ,2 ,3],
                                 [-1,0 ,1, 1],
                                 [-2,-1 ,0 ,1],
                                 [-1,-2,-1,0]]) * (np.pi/2)
        self.b_turn_ccw = np.array([[0 ,1 ,0 ,1],
                                    [-1,0 ,1,0],
                                    [0,-1 ,0 ,1],
                                    [-1,0,-1,0]]) * (np.pi/2)

        self.R = self.R_matrix(0) # intermediate term for update math
        self.inter_sum = np.zeros(2)


        #==Dynamic Variables==#
        self.state = np.zeros((2,self.N))
        self.freq = np.zeros(self.N)
        self.dstate = np.zeros((2,self.N))
        self.radius = np.zeros(self.N)
        self.phase = np.zeros(self.N)
        self.phase_off = np.zeros(self.N) # track rotations to give continuous phase output
        self.x_last = np.zeros(self.N)
        self.d_rot = np.zeros(self.N) # delta terms for wheel feedback
        self.d_ext = np.zeros(self.N)
        self.d_biases = np.zeros((self.N,self.N)) # delta for phase bias updates

        # filtered extension dynamics
        self.off_tar = np.zeros(self.N)
        self.off = np.zeros(self.N)
        self.d_off = np.zeros(self.N)
        self.off_gain = 2.0

        # feedback term applied to both state variables
        self.feedback = np.zeros(2)
        self.disturbance = np.zeros((2,self.N))

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
        self.freq_beta = 1.0
        self.random_state = np.random.RandomState(111)

    def R_matrix(self,theta):
        return  np.array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])

    def euler_update(self, t_step):
        # phase and extension offset dynamics
        self.bias_inter += self.d_biases * t_step
        # self.d_off = self.off_gain * (.25 * self.off_gain * (self.off_tar-self.off))
        # self.off += self.d_off

        for i in range(self.N):
            phi = np.arctan(self.state[1,i]/self.state[0,i])
            #exp_m = np.exp(-1.0 *self.state[0,:] * self.freq_beta)
            #exp_p = np.exp(self.state[0,:] * self.freq_beta)
            self.freq[i] = self.freq_const[i]

            self.radius[i] = self.state[0,i]**2 + self.state[1,i]**2

            A = 50 / (np.sqrt(self.amplitudes[i]) * phi**2)

            # coupling term applied based on relative phase offset
            self.inter_sum = np.zeros(2)
            for j in range(self.N):
                self.R = self.R_matrix(self.bias_inter[i,j])
                self.inter_sum += self.weights_inter[i,j] * np.matmul(self.R,self.state[:,j])

            # orbit dynamics
            # dx/dt = a*(u-r^2)*x - w*y
            self.dstate[0,i] = (A * (self.amplitudes[i] * phi**2 - self.radius[i] ** 2) * self.state[0,i]
                                - self.freq[i] * phi * self.state[1, i])# + self.inter_sum[0])
            # dy/dt = a*(u-r^2)*y - w*x
            self.dstate[1, i] = (A * (self.amplitudes[i] * phi**2 - self.radius[i] ** 2) * self.state[1, i]
                                 + self.freq[i] * phi * self.state[0, i])# + self.inter_sum[1])

            # apply derivatives by euler method
            self.x_last[i] = 1.0*self.state[1, i]  # track last state for phase check
            self.state[:,i] += self.dstate[:,i] * t_step

        # update phase and phase offset
        self.set_phase_offset()
        for i in range(self.N):
            self.phase[i] = arctan2(self.state[1, i], self.state[0, i]) + self.phase_off[i]


    def set_phase_offset(self):
    # run anytime states are updated to track quadrant changes for smooth arctan
        for i in range(self.N):
            if self.state[0, i] < 0:
                if self.state[1, i] < 0.0 and self.x_last[i] > 0.0:
                    self.phase_off[i] += 2 * pi
                if self.state[1, i] > 0.0 and self.x_last[i] < 0.0:
                    self.phase_off[i] -= 2 * pi

    def set_state(self, n, state):
        self.state[:, n] = state

    def wheel_feedback(self,rot,ext):
        for i in range(self.N):
            self.d_rot = self.phase[i] - rot[i]
            self.d_ext = self.radius[i] - ext[i]
        pass

    def wheel_output(self):
        # return current phase and extension
        ext = (self.radius-1.0) + (self.state[0,:]-1.0)*self.ext_amp*0.5
        rot = self.phase/self.n_arc
        return rot.tolist(),[max(e,0.0) for e in ext]

    def perturbation(self, n, sigmas):
        noise_x = self.random_state.randn(self.N) * sigmas[0]
        noise_y = self.random_state.randn(self.N) * sigmas[1]
        self.x_last[:] = self.state[1,:]
        self.state += [noise_x,noise_y]
        self.set_phase_offset()

    def graph_output(self):
        return self.state[1,:]

    def diff_input(self,v,w,h):
        # implementing a pseudo-differential drive controller

        # wheel speed ~= oscillator frequency, get from diff drive kinematics
        differential = (w * self.wheel_dist / self.wheel_rad)
        self.freq_const[:] = ((v * self.n_arc / self.wheel_rad) + self.wheel_dir * differential)
        # phase biases must change over time to compensate for differential motion
        self.d_biases = self.b_turn_ccw * differential

        # get phase difference from requested ride height, only calculate if height changes
        # *** this assumes wheels are all the same to lower computation time ***
        if h != self.height:
            self.height = h
            # phase difference sent to control is relative to completely closed position
            p,p_l = self.wheels[0].p_closed - self.wheels[0].calc_phase_diff(h)
            self.amplitudes[:] = p + 1.0
            print('ph diffs : ',p,p_l)
            self.ext_amp[:] = abs(p_l - p)


class GeneratorHopf(CPG):
    def __init__(self, num_oscillators : int, robot : RobotDefinition):
        #==Constant Parameters==#
        self.N = num_oscillators
        self.freq_const = np.zeros(self.N)
        self.weights_converge = np.ones((2,self.N))
        self.amplitudes = np.ones(self.N)
        self.weights_inter = np.ones((self.N,self.N))
        self.bias_inter = np.zeros((self.N,self.N))
        self.freq_off = np.ones(self.N) * 0.1
        self.ext_gains = np.ones(self.N) * 0.1
        self.ext_amp = np.zeros(self.N)

        # phase bias arrays for starting and turning
        self.b_q_off = np.array([[0 ,1 ,2 ,3],
                                 [-1,0 ,1, 1],
                                 [-2,-1 ,0 ,1],
                                 [-1,-2,-1,0]]) * (np.pi/2)
        self.b_turn_ccw = np.array([[0 ,1 ,0 ,1],
                                    [-1,0 ,1,0],
                                    [0,-1 ,0 ,1],
                                    [-1,0,-1,0]])

        self.R = self.R_matrix(0) # intermediate term for update math
        self.inter_sum = np.zeros(2)


        #==Dynamic Variables==#
        self.state = np.zeros((2,self.N))
        self.freq = np.zeros(self.N)
        self.dstate = np.zeros((2,self.N))
        self.radius = np.zeros(self.N)
        self.phase = np.zeros(self.N)
        self.phase_off = np.zeros(self.N) # track rotations to give continuous phase output
        self.x_last = np.zeros(self.N)
        self.d_rot = np.zeros(self.N) # delta terms for wheel feedback
        self.d_ext = np.zeros(self.N)
        self.d_biases = np.zeros((self.N,self.N)) # delta for phase bias updates

        # filtered extension dynamics
        self.off_tar = np.zeros(self.N)
        self.off = np.zeros(self.N)
        self.d_off = np.zeros(self.N)
        self.off_gain = 2.0

        # feedback term applied to both state variables
        self.feedback = np.zeros(2)
        self.disturbance = np.zeros((2,self.N))

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
        self.freq_beta = 1.0
        self.random_state = np.random.RandomState(111)

    def R_matrix(self,theta):
        return  np.array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])

    def euler_update(self, t_step):
        # phase and extension offset dynamics
        self.bias_inter += self.d_biases * t_step
        # self.d_off = self.off_gain * (.25 * self.off_gain * (self.off_tar-self.off))
        # self.off += self.d_off

        for i in range(self.N):
            #exp_m = np.exp(-1.0 *self.state[0,:] * self.freq_beta)
            #exp_p = np.exp(self.state[0,:] * self.freq_beta)
            self.freq[i] = self.freq_const[i]

            self.radius[i] = self.state[0,i]**2 + self.state[1,i]**2

            # coupling term applied based on relative phase offset
            self.inter_sum = np.zeros(2)
            for j in range(self.N):
                self.R = self.R_matrix(self.bias_inter[i,j])
                self.inter_sum += self.weights_inter[i,j] * np.matmul(self.R,self.state[:,j])

            # orbit dynamics
            # dx/dt = a*(u-r^2)*x - w*y
            self.dstate[0,i] = (self.weights_converge[0,i] * (self.amplitudes[i] - self.radius[i] ** 2) * self.state[0,i]
                                - self.freq[i] * self.state[1, i] + self.inter_sum[0])
            # dy/dt = a*(u-r^2)*y - w*x
            self.dstate[1, i] = (self.weights_converge[0,i] * (self.amplitudes[i] - self.radius[i] ** 2) * self.state[1, i]
                                 + self.freq[i] * self.state[0, i] + self.inter_sum[1])

            # apply derivatives by euler method
            self.x_last[i] = 1.0*self.state[1, i]  # track last state for phase check
            self.state[:,i] += self.dstate[:,i] * t_step

        # update phase and phase offset
        self.set_phase_offset()
        for i in range(self.N):
            self.phase[i] = arctan2(self.state[1, i], self.state[0, i]) + self.phase_off[i]


    def set_phase_offset(self):
    # run anytime states are updated to track quadrant changes for smooth arctan
        for i in range(self.N):
            if self.state[0, i] < 0:
                if self.state[1, i] < 0.0 and self.x_last[i] > 0.0:
                    self.phase_off[i] += 2 * pi
                if self.state[1, i] > 0.0 and self.x_last[i] < 0.0:
                    self.phase_off[i] -= 2 * pi

    def set_state(self, n, state):
        self.state[:, n] = state

    def wheel_feedback(self,rot,ext):
        for i in range(self.N):
            self.d_rot = self.phase[i] - rot[i]
            self.d_ext = self.radius[i] - ext[i]
        pass

    def wheel_output(self):
        # return current phase and extension
        ext = (self.radius-1.0) + (self.state[0,:]-1.0)*self.ext_amp*0.5
        rot = self.phase/self.n_arc
        return rot.tolist(),[max(e,0.0) for e in ext]

    def perturbation(self, n, sigmas):
        noise_x = self.random_state.randn(self.N) * sigmas[0]
        noise_y = self.random_state.randn(self.N) * sigmas[1]
        self.x_last[:] = self.state[1,:]
        self.state += [noise_x,noise_y]
        self.set_phase_offset()

    def graph_output(self):
        return self.state[1,:]

    def diff_input(self,v,w,h):
        # implementing a pseudo-differential drive controller

        # wheel speed ~= oscillator frequency, get from diff drive kinematics
        differential = (w * self.wheel_dist / self.wheel_rad)
        self.freq_const[:] = ((v * self.n_arc / self.wheel_rad) + self.wheel_dir * differential)
        # phase biases must change over time to compensate for differential motion
        self.d_biases = self.b_turn_ccw * differential

        # get phase difference from requested ride height, only calculate if height changes
        # *** this assumes wheels are all the same to lower computation time ***
        if h != self.height:
            self.height = h
            # phase difference sent to control is relative to completely closed position
            p,p_l = self.wheels[0].p_closed - self.wheels[0].calc_phase_diff(h)
            self.amplitudes[:] = p + 1.0
            print('ph diffs : ',p,p_l)
            self.ext_amp[:] = abs(p_l - p)