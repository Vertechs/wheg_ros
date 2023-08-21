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

    def reset_oscillators(self,n):
        """
        Reset the states of oscillators in the network
        :param n: list of oscillators to reset, n=-1 to reset all
        :return:
        """
        raise NotImplementedError

    def command_input(self,v : np.ndarray,e : np.ndarray, n_arc : int, radius_ratio):
        """
        Apply commanded wheel velocities and extensions to CPG paramters
        :param v: velocities of each wheel, size should match number of oscillators
        :param e: extensions of each wheel, "^   "
        :param n_arc: number of arcs in wheel, used to scale oscillator frequency
        :return:
        """
        raise NotImplementedError

    def wheel_output(self,n_arc):
        """
        Send wheel position commands as lists of rotation and extension values
        # TODO better way to handle number of arcs
        :return: rot,ext
        :rtype rot: list
        :rtype ext: list
        """
        raise NotImplementedError

    def wheel_feedback(self,rot,ext):
        """
        Update internal states or apply feedback signal using wheel position estimates passed up from controller
        :param rot: array or list of phase position values for each wheel
        :param ext: array or list of extension amounts for each wheel
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

    def wheel_output(self, n_arc):
        values = self.amp * np.sing(self.phi) + self.off
        return self.phi, values

    def command_input(self,v : np.ndarray,e : np.ndarray, n_arc : int, radius_ratio):
        for i in range(self.N):
            self.own_freq[i] = v[i] * n_arc
            self.a




# Implement Matsuoka oscillator network
# Model retrieved from: DOI:10.1109/GCIS.2012.99
class GeneratorMatsuokaNet(CPG):
    def __init__(self,num_oscillators,default_weight=1.0,default_frequency=1.0):
        # define constant parameters
        self.N = num_oscillators
        self.time_constants = np.ones((2,self.N)) * 1/default_frequency
        # intra-oscillator weights
        self.w_own = np.ones((2, self.N)) * default_weight # neuron self-inhibition
        self.w_mut = np.ones((2, self.N)) * default_weight # opposing neuron inhibition
        # weights of influence between oscillators, hollow (-)ones matrix by default
        self.w_inter = -1.0 * (np.ones((self.N, self.N)) - np.eye(self.N)) * default_weight

        # define dynamic variables
        self.u = np.zeros((2,self.N))
        self.v = np.zeros((2,self.N))
        self.y = np.zeros((2,self.N))
        self.s = 0.0
        self.du = np.zeros((2,self.N))
        self.dv = np.zeros((2,self.N))

        self.feed = np.zeros(self.N)

    def euler_update(self,t_step):
        # calculate neuron outputs for all neurons
        for i in range(self.N):
            # calculate for extensor and flexor neuron
            for e in [0,1]:
                self.y[e,i] = max(0, self.u[e,i])

        # calculate state derivatives for each oscillator
        for i in range(self.N):
            # calculate for extensor and flexor neuron
            for e in [0, 1]:
                # get index of other neuron in own oscillator: [1,0]
                f = int(not e)

                # calculate weighted sum of other oscillator influence
                inter_sum = 0
                for j in range(self.N):
                    inter_sum += self.w_inter[i, j] * self.y[e, j]

                # calculate first state (u) derivative
                self.du[e,i] = ((-self.u[e,i]
                               - self.w_mut[e,i]*self.y[f,i]
                               - self.w_own[e,i]*self.v[e,i]
                               + inter_sum + self.feed[i] + self.s)
                               / self.time_constants[0,i])

                # calculate second state (v) derivative
                self.dv[e,i] = (-self.v[e,i] + self.y[e,i]) / self.time_constants[1,i]

        # apply derivatives
        self.u += self.du*t_step
        self.v += self.dv*t_step

    def set_state_all(self,state):
        self.u[0,:] = state[0,:]
        self.u[1,:] = state[1,:]
        self.v[0,:] = state[2,:]
        self.v[1,:] = state[3,:]

    def set_state_single(self,n,u1,v1,u2,v2):
        self.u[0, n] = u1
        self.v[0, n] = v1
        self.u[1, n] = u2
        self.v[1, n] = v2

    def set_input(self,n,in_val):
        self.s = in_val

    def graph_output(self):
        return self.y[1,:] - self.y[0,:]

    def wheel_output(self):
        return np.max(self.y,axis=0)

    def state_output(self):
        return np.vstack([self.u,self.v])




# Implement separate Matsuoka oscillators with two neurons each
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
        return self.output[1,:] - self.output[0,:]


# Implementing Hopf oscillator with variable speed
# TODO add source (currently DOI:10.1109/ROBOT.2008.4543306)
class GeneratorHopfMod(CPG):
    def __init__(self, num_oscillators, default_weight=1.0, default_frequency=1.0):
        #==Constant Parameters==#
        self.N = num_oscillators
        self.freq = np.zeros((1,self.N))
        self.weights_converge = np.ones((2,self.N))
        self.amplitude = np.ones((1,self.N))
        self.weights_inter = np.ones((self.N,self.N))

        #==Dynamic Variables==#
        self.state = np.zeros((2,self.N))
        self.dstate = np.zeros((2,self.N))
        self.radius = np.zeros((1,self.N))

        # input applied only to the second state variable (y)
        self.input = 0.0


    def euler_update(self, t_step):
        for i in range(self.N):
            # TODO add frequency shaping math
            # self.freq =
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
        return np.vstack(self.radius,np.arctan2(self.state[1,:],self.state[0,:]))

    def graph_output(self):
        return self.state[0,:]

class GeneratorVdpNet(CPG):
    def __init__(self,num_oscillators):
        # Static parameters
        self.N = num_oscillators
        self.para = np.ones((3,self.N)) # a, p^2, w^2
        # weights should be small compared to amplitude
        self.weights = (np.ones((self.N,self.N)) - np.eye(self.N)) * -0.2

        # Dynamic variables
        self.x = np.zeros(self.N)
        self.y = np.zeros(self.N)
        self.dx = np.zeros(self.N)
        self.dy = np.zeros(self.N)

    def euler_update(self,t_step):
        for i in range(self.N):
            inter_sum = self.x[i]
            for j in range(self.N):
                inter_sum += self.weights[i,j] * self.x[j]

            self.dy[i] = (self.para[0,i] * (self.para[1,i] - inter_sum**2) * self.dx[i]
                          - self.para[2,i] * inter_sum)
            self.dx[i] = self.y[i]

        self.x += self.dx*t_step
        self.y += self.dy*t_step

    def set_state(self,n,x,y):
        self.x[n] = x
        self.y[n] = y

    def state_output(self,n):
        return np.vstack([self.x[n],self.y[n]])

    def phase_output(self):
        # return x component and current phase
        return np.vstack(self.x,np.arctan2(self.x,self.y))

    def graph_output(self):
        return self.x




