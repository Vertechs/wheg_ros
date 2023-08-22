import numpy as np

from wheg_utils.generators.central_pattern_generator import CPG


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
