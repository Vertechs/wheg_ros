import numpy as np


# Implement fully connected Kurumato oscillator network
class CPG_FIC:
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

    def euler_update(self, time_step, output_func):
        # eulers method, y(t+1) = y(t) + T*h(t,y)
        T = time_step
        outN = np.size(output_func(0,0,0)) # BAD
        theta = np.zeros((self.N,outN))

        # for each oscillator
        for i in range(self.N):

            # PHI UPDATE // get delta phase / dt
            delphi = self.own_freq[i]
            for j in range(self.N):
                if j != i: # no self weights
                    inner = self.phi[j] - self.phi[i] - self.biases[i,j]
                    inner = np.sin(inner)
                    delphi += self.weights[i,j] * self.amp[j] * inner
                    #print(i,j,delphi)
            self.phi[i] += T * delphi

            # AMP UPDATE // delta^2 amplitude /d2t
            inner = 0.25 * self.gain_amp * (self.target_amps[i]-self.amp[i])
            ddelamp = self.gain_amp * (inner - self.d_amp[i])
            self.d_amp[i] += T * ddelamp
            self.amp[i] += T * self.d_amp[i]

            # OFF UPDATE // delta^2 offsets /d2t
            inner = 0.25 * self.gain_off * (self.target_offs[i] - self.off[i])
            ddeloff = self.gain_off * (inner - self.d_off[i])
            self.d_off[i] += T * ddeloff
            self.off[i] += T * self.d_off[i]

            # OUTPUT // return signal
            theta[i,:] = output_func(self.off[i],self.amp[i],self.phi[i])

        return theta

    def default_callback(self,off,amp,phi):
        return off + amp * np.sin(phi)

    def reset_oscillator(self):
        self.phi = np.zeros(self.N)
        self.amp = self.target_amps
        self.off = self.target_offs

    def pertubation(self,sigmas,random_state=np.random.RandomState(111)):
        # gaussian additive noise
        phi_noise = random_state.randn(self.N) * sigmas[0]
        amp_noise = random_state.randn(self.N) * sigmas[1]
        off_noise = random_state.randn(self.N) * sigmas[2]

        self.phi += phi_noise
        self.amp += amp_noise
        self.off += off_noise

    def output(self):
        # calculate output without updating state
        theta = np.zeros(self.N)
        for i in range(self.N):
            theta[i] = self.off[i] + self.amp[i] * np.sin(self.phi[i])
        return theta


