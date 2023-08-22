import numpy as np

from wheg_utils.generators.central_pattern_generator import CPG


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
