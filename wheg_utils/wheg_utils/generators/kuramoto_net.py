import numpy as np

from wheg_utils.generators.central_pattern_generator import CPG
from wheg_utils.four_bar_wheg_circ import WhegFourBar
from wheg_utils.robot_config import RobotDefinition

class GeneratorKuramoto(CPG):
    def __init__(self, num_oscillator : int, robot : RobotDefinition):
        self.N = num_oscillator
        n = self.N

        # init state vectors
        self.phi = np.zeros(n)
        self.amp = np.zeros(n)
        self.off = np.zeros(n)
        self.own_freq = np.ones(n)

        # init dt state vectors
        self.d_amp = np.zeros(n)
        self.d_off = np.zeros(n)
        self.d_freq = np.zeros(n)

        # reference weight matrices to interpolate between
        self.w_full = np.ones((n, n)) - np.diag(np.ones((1, n)))
        self.w_half = np.ones((n, n)) - np.diag(np.ones((1, n)))
        self.b_walk = np.array([[0 ,2 ,1 ,3],
                                 [-2,0 ,-1,1],
                                 [-1,1 ,0 ,2],
                                 [-1,-1,-2,0]]) * (np.pi/4)
        self.b_q_off = np.array([[0 ,1 ,2 ,3],
                                 [-1,0 ,1, 1],
                                 [-2,-1 ,0 ,1],
                                 [-1,-2,-1,0]]) * (np.pi/4)
        self.b_turn_ccw = np.array([[0 ,1 ,0 ,1],
                                    [-1,0 ,1,0],
                                    [0,-1 ,0 ,1],
                                    [-1,0,-1,0]])
        self.freq_ccw = np.array([-1, 1, -1, 1])  # wheel directions for ccw turning

        # relational parameters and rates of change
        self.weights = self.w_full
        self.biases = np.zeros((n,n))

        self.d_biases = np.zeros((n,n))
        self.d_weights = np.zeros((n,n))

        # control parameters
        self.freq_tar = np.zeros(n)
        self.target_amps = np.ones(n)
        self.target_offs = np.zeros(n)
        self.amp_norm = np.ones(n) * 5.0

        # feedback variables
        self.d_rot = np.zeros(n)
        self.d_ext = np.zeros(n)

        # gain parameters, how fast converges
        self.gain_off = 2.0 # rad/s
        self.gain_amp = 2.0 # rad/s
        self.freq_gain = 1.0
        self.feed_gain = -0.2

        self.random_state = np.random.RandomState(111)

        # pseudo-differential drive parameters
        # need wheg objects for each oscillator to get basic parameters and do some inverse kinematics
        self.wheels = []
        self.n_arc = []
        for i in range(self.N):
            self.wheels.append(WhegFourBar(robot.modules[i].four_bar.get_parameter_list()))
        self.wheel_dist = robot.wheel_base_width
        self.n_arc = robot.modules[0].n_arc
        self.wheel_rad = robot.modules[0].radius
        self.height = 0.0
        self.rot_out_offset = self.n_arc

    def euler_update(self, time_step):
        # eulers method, y(t+1) = y(t) + T*h(t,y)
        # integrate changes in bias matrix
        self.biases += self.d_biases * time_step

        # for each oscillator
        for i in range(self.N):
            # FREQUENCY UPDATE
            #dd_freq = (0.25 * self.freq_gain * (self.freq_tar[i] - self.own_freq[i]) - self.d_freq[i]) * self.freq_gain
            self.d_freq[i] = self.freq_gain * (self.freq_tar[i] - self.own_freq[i]) #+= dd_freq * time_step #
            self.own_freq[i] += self.d_freq[i] * time_step

            # PHI UPDATE // get (delta_phase / dt)
            delphi = self.own_freq[i]
            for j in range(self.N):
                if j != i: # no self weights
                    inner = self.phi[j] - self.phi[i] - self.biases[i,j]
                    inner = np.sin(inner)
                    inner = self.weights[i,j] * inner # amplitude term not included
                    delphi += inner
            #delphi += self.d_rot[i] * self.feed_gain

            self.phi[i] += time_step * delphi

            # AMP UPDATE // (delta^2_amplitude / d^2t)
            inner = 0.25 * self.gain_amp * (self.target_amps[i]-self.amp[i])
            ddelamp = self.gain_amp * (inner - self.d_amp[i])
            self.d_amp[i] += time_step * ddelamp
            self.amp[i] += time_step * self.d_amp[i]

            # OFF UPDATE // (delta^2_offsets / d^2t)
            inner = 0.25 * self.gain_off * (self.target_offs[i] - self.off[i])
            ddeloff = self.gain_off * (inner - self.d_off[i])
            self.d_off[i] += time_step * ddeloff
            self.off[i] += time_step * self.d_off[i]

    def reset_oscillators(self):
        self.phi = np.zeros(self.N)
        self.amp = self.target_amps
        self.off = self.target_offs
        self.freq = self.freq_tar

    def perturbation(self,n,sigmas):
        # gaussian additive noise
        phi_noise = self.random_state.randn(self.N) * sigmas[0]
        amp_noise = self.random_state.randn(self.N) * sigmas[1]
        off_noise = self.random_state.randn(self.N) * sigmas[2]

        self.phi += phi_noise
        self.amp += amp_noise
        self.off += off_noise

    def graph_output(self):
        # calculate output without updating state
        theta = np.zeros(self.N)
        for i in range(self.N):
            theta[i] = self.off[i] + self.amp[i] * np.sin(self.phi[i])
        return theta

    def wheel_output(self):
        # self.ext_out = self.off + self.amp * np.sin(self.phi)
        # self.rot_out = self.phi / self.n_arc
        self.ext_out = self.off - np.abs(self.amp * np.sin(self.phi))
        self.rot_out = 2 * self.phi / self.n_arc
        return self.rot_out.tolist(),[max(0.0,s) for s in self.ext_out]

    def wheel_feedback(self,rot,ext):
        # adjust internal states based on estimated position and last commanded position
        self.d_rot = self.rot_out - rot
        # only change the offset state; leaving amplitude as a completely internal state
        self.d_ext = self.ext_out - ext

    def diff_input(self, v, w, h):
        # implementing a pseudo-differential drive controller

        # wheel speed ~= oscillator frequency, get from diff drive kinematics
        differential = (w * self.wheel_dist / self.wheel_rad)
        self.freq_tar[:] = ((v / self.wheel_rad) + self.freq_ccw * differential)
        #print('freq:',self.freq_tar)
        # phase biases change over time to induce differential motion
        self.d_biases = self.b_turn_ccw * differential * 2

        # get phase difference from requested ride height, only calculate if height changes
        # *** currently assumes wheels are all the same to lower computation time ***
        if h != self.height:
            self.height = h

            # set desired radius/offset as the extension one half step forward
            # set oscillation amplitude as the difference between half step and bottom dead center
            # phase difference sent to control is relative to completely closed position
            ph, pb = self.wheels[0].calc_phase_diffs(h)
            self.target_offs[:] = self.wheels[0].p_closed - ph # desired radius
            print('ph diffs : ',ph,pb)
            print(self.wheels[0].p_closed)
            self.target_amps[:] = ph - pb  # oscillation amplitudes





