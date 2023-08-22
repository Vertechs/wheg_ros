import numpy as np

from wheg_utils.generators.central_pattern_generator import CPG


# Implement Matsuoka oscillator network
# Model from: DOI:10.1109/GCIS.2012.99

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
