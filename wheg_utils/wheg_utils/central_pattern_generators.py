import numpy as np

class CPG:
    #TODO fill out abstract class
    def euler_update(self,t_step):
        """
        Calculate new states and derivatives, update states according to Eulers method
        :param t_step: time step to update over
        """
        raise NotImplementedError

    def graph_output(self):
        """
        :return: array of floats with size = (N_oscillator,1)
        :rtype: np.ndarray
        """
        raise NotImplementedError

    def perturbation(self, n, sigmas):
        """
        Apply a random perturbation to the states of an oscillator
        :param sigmas: array of std_deviation for each state
        :param n: oscillator to apply the perturbation to
        """
        raise NotImplementedError

    def reset_oscillators(self,n=0):
        """
        Reset the states of oscillators in the network
        :param n: list of oscillators to reset, n=0 to reset all
        :return:
        """
        raise NotImplementedError

# Implement fully connected Kuramoto oscillator network TODO add reference
class GeneratorKuramoto(CPG):
    def __init__(self, num_oscillator, default_weight=1.0, default_frequency=1.0):
        self.N = num_oscillator
        n = self.N

        # init state vectors
        self.phi = np.zeros(n)
        self.amp = np.zeros(n)
        self.off = np.zeros(n)

        # init dt state vectors
        self.d_amp = np.zeros(n)
        self.d_off = np.zeros(n)

        # relational parameters
        self.weights = np.ones((n,n)) - np.diag(np.ones((1,n)))
        self.weights = self.weights * default_weight
        self.biases = np.zeros((n,n))

        # control parameters
        self.own_freq = np.ones(n)*default_frequency
        self.target_amps = np.ones(n)
        self.target_offs = np.zeros(n)

        # gain parameters
        self.gain_off = 2.0 # rad/s
        self.gain_amp = 2.0 # rad/s

        self.random_state = np.random.RandomState(111)

    def euler_update(self, time_step):
        # eulers method, y(t+1) = y(t) + T*h(t,y)
        T = time_step
        
        # for each oscillator
        for i in range(self.N):

            # PHI UPDATE // get (delta_phase / dt)
            delphi = self.own_freq[i]
            for j in range(self.N):
                if j != i: # no self weights
                    inner = self.phi[j] - self.phi[i] - self.biases[i,j]
                    inner = np.sin(inner)
                    inner = self.weights[i,j] * self.amp[j] * inner
                    delphi += inner

            self.phi[i] += T * delphi

            # AMP UPDATE // (delta^2_amplitude / d^2t)
            inner = 0.25 * self.gain_amp * (self.target_amps[i]-self.amp[i])
            ddelamp = self.gain_amp * (inner - self.d_amp[i])
            self.d_amp[i] += T * ddelamp
            self.amp[i] += T * self.d_amp[i]

            # OFF UPDATE // (delta^2_offsets / d^2t)
            inner = 0.25 * self.gain_off * (self.target_offs[i] - self.off[i])
            ddeloff = self.gain_off * (inner - self.d_off[i])
            self.d_off[i] += T * ddeloff
            self.off[i] += T * self.d_off[i]

    def reset_oscillator(self):
        self.phi = np.zeros(self.N)
        self.amp = self.target_amps
        self.off = self.target_offs

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

# Implement Matsuoka oscillator network
# Model retrieved from: DOI:10.1109/GCIS.2012.99
class GeneratorMatsuoka(CPG):
    def __init__(self,num_oscillators,default_weight=1.0,default_frequency=1.0):
        # define constant parameters
        self.N = num_oscillators
        self.time_constants = np.ones((2,self.N)) * 1/default_frequency
        self.w_own = np.ones((2, self.N)) * default_weight
        self.w_mut = np.ones((2, self.N)) * default_weight
        self.w_btwn = (np.ones((self.N,self.N)) - np.eye(self.N))*default_weight

        # define dynamic variables
        self.u = np.zeros((2,self.N))
        self.v = np.zeros((2,self.N))
        self.y = np.zeros((2,self.N))
        self.s = 0.0
        self.du = self.u
        self.dv = self.v

    def euler_update(self,t_step):
        # calculate neuron outputs for all neurons
        for i in range(self.N):
            # calculate for extensor and flexor neuron
            for e in [0,1]:
                self.y[e,i] = max(0, self.u[e,i])

        # calculate state derivatives
        for i in range(self.N):
            # calculate for extensor and flexor neuron
            for e in [0, 1]:
                # calculate weighted sum of other oscillator influence
                btwn_sum = 0
                for j in range(self.N):
                    btwn_sum += self.w_btwn[i,j]*self.y[e,j]

                # get index of other neuron in own oscillator: [1,0]
                if e == 1: f=0
                else: f=1

                # calculate second state (v) derivative
                self.dv[e,i] = -self.v[e,i] + self.y[e,i]

                # calculate first state (u) derivative
                self.du[e,i] = - self.u[e,i] \
                               - self.w_mut[e,i]*self.y[f,i] \
                               - self.w_own[e,i]*self.v[e,i] \
                               - btwn_sum - self.s
        # apply derivatives
        self.u += self.du*t_step
        self.v += self.dv*t_step

    def set_state(self,n,u1,v1,u2,v2):
        self.u[0, n] = u1
        self.v[0, n] = v1
        self.u[1, n] = u2
        self.v[1, n] = v2

    def set_input(self,n,in_val):
        self.s = in_val

    def graph_output(self):
        return [wye[0]-wye[1] for wye in self.y.tolist(axis=0)]

# Implement separate Matsuoka oscillators with two neurons
class GeneratorMatsuokaSingle(CPG):
    def __init__(self,num_oscillators,default_weight=1.0,default_frequency=1.0):
        # define constant parameters
        self.N = num_oscillators
        self.time_constants = np.ones((2,self.N)) * 1/default_frequency
        self.weights_own = np.ones((2, self.N)) * default_weight #B
        self.weights_mut = np.ones((2, self.N)) * default_weight #W

        # define dynamic variables
        self.state = np.zeros((4,self.N))  # [u1, u2, v1, v2]^T
        self.dstate = np.zeros((4,self.N)) # d_state/dt
        self.output = np.zeros((2,self.N)) # [y1, y2]^T

        self.input = np.zeros((1,self.N))  # [s0]

    def euler_update(self,t_step):
        for n in range(self.N):
            for i in [0,1]:
                # calculate neuron output
                self.output[i, n] = max(0, self.state[i, n])

            for i in [0,1]:
                j = [1,0] # use to select output from other neuron
                # first 2 state derivatives (potential terms)
                # T*u_dot(i) = -u(i) - w(ij)*y(j) - b(i)*v(i) + s0
                self.dstate[i,n] = ((- self.state[i,n]
                                   - self.output[j[i],n] * self.weights_mut[i, n]
                                   - self.state[i+2, n] * self.weights_own[i,n]
                                   + self.input)
                                   / self.time_constants[0,n])

            # second 2 state derivatives (inhibitor terms)
            for i in [2,3]:
                self.dstate[i,n] = (-self.state[i,n] + self.output[i-2,n]) / self.time_constants[1,n]

        # apply derivatives
        self.state += self.dstate*t_step

    def set_state(self,n,state):
        self.state[:,n] = state

    def set_input(self,n,in_val):
        self.input[0,n] = in_val

    def graph_output(self):
        return [np.max(u) for u in self.state[0:2]]


