import numpy as np
import matplotlib.pyplot as plt

import wheg_utils.generators.kuramoto_net
import wheg_utils.generators.matsuoka_net
import wheg_utils.generators.mod_hopf_net_turbo
import wheg_utils.generators.van_der_pol_net

from wheg_utils import robot_config
robot = robot_config.get_config_A()

## Mastuoka oscillator setup
gen1 = wheg_utils.generators.matsuoka_net.GeneratorMatsuokaNet(4)

gen1.time_constants[0, :] = 0.4
gen1.time_constants[1, :] = 4.0
gen1.set_input(0, 4.0)

gen1.w_own[:, :] = np.ones((2,4)) * 2.0
gen1.w_mut[:, :] = np.ones((2,4)) * 2.5

gen1.set_state_all(np.array([[-0.54,0.38,0.21,-0.58],
                             [0.16,0.11,0.22,0.10],
                             [0.38,-0.54,-0.58,0.21],
                             [0.11,0.16,0.10,0.22]]).T)
# gen1.set_state_single(0,0.0111,0.002,0.0081,0.0057)


## Kuramoto oscillator setup
gen2 = wheg_utils.generators.kuramoto_net.GeneratorKuramoto(4,robot)

gen2.random_state = np.random.RandomState(23)

gen2.gain_amp = 2.0

gen2.target_amps = np.array([0.5, 0.5, 0.5, 0.5])
gen2.target_offs = np.zeros(4)
gen2.own_freq[:] = 1.0

gen2.weights = 0.0*(np.ones((4, 4)) - np.eye(4))
gen2.biases[:] = gen2.b_q_off

gen2.perturbation(0,[.01, .01, .01])
gen2.diff_input(10.0, 0.0, gen2.wheel_rad)


## Modified hopf oscillators
gen3 = wheg_utils.generators.mod_hopf_net.GeneratorHopf(4,robot)

gen3.freq_const = np.ones(gen3.N)
gen3.weights_converge = np.vstack([np.ones((1,4))*5,np.ones((1,4))*5])
gen3.amplitudes = np.ones(gen3.N)

# gen3.weights_inter = np.array([[0,-1,1,-1],
#                                [-1,0,-1,1],
#                                [-1,1,0,-1],
#                                [1,-1,-1,0]])
gen3.weights_inter = 0.5 * (np.ones((gen3.N,gen3.N)) - np.eye(gen3.N))
gen3.bias_inter = gen3.b_q_off

for i in range(4):
    gen3.set_state(i,[0.2,0.01])

gen3.diff_input(10.0,0.0,gen3.wheel_rad*1.5)


## Van der pol coupled oscillators
gen4 = wheg_utils.generators.van_der_pol_net.GeneratorVdpNet(4,robot)

A = np.array([1,2.0,10])[np.newaxis]
gen4.para = np.tile(A.T,gen4.N)

gen4.weights = np.array([[0,-1,1,-1],
                       [-1,0,-1,1],
                       [-1,1,0,-1],
                       [1,-1,-1,0]]) * 0.2

for i in range(4):
    gen4.set_state(i,0.01*i,0.01)


## Generating and graphing CPG outputs
t_max = 40
t_step = 0.005
max_iter = int(t_max/t_step)

x = np.linspace(0,max_iter*t_step,max_iter)
y = []

for i in range(6):
    y.append(np.zeros((4,max_iter)))

fig = plt.figure(figsize=(18,7))
axs = []
for i in range(6):
    axs.append(fig.add_subplot(321+i))

titles = ['K phs','K ext','H phs','H ext','V phs','V ext']
plt.ion()

px = np.zeros((max_iter,4))
py = np.zeros((max_iter,4))

for t in range(max_iter):
    #gen1.euler_update(t_step)
    gen2.euler_update(t_step)
    gen3.euler_update(t_step)
    gen4.euler_update(t_step)

    y[0][:,t] = gen2.wheel_output()[0]
    y[1][:,t] = gen2.wheel_output()[1]
    y[2][:,t] = gen3.wheel_output()[0]
    y[3][:,t] = gen3.wheel_output()[1]
    y[4][:,t] = gen4.wheel_output()[0]
    y[5][:,t] = gen4.wheel_output()[1]

    px[t] = gen3.state[0,:]
    py[t] = gen3.state[1,:]

    #px[t] = gen4.x
    #py[t] = gen4.y

    if t == int(10/t_step):
        gen2.diff_input(10.0,0.0,gen2.wheel_rad*1.5)
        gen3.diff_input(10.0,0.0,gen3.wheel_rad*1.5)

    if t == int(20/t_step):
        gen2.diff_input(10.0,0.1,gen2.wheel_rad * 1.5)
        gen3.diff_input(10.0,0.1,gen3.wheel_rad * 1.5)
        gen4.diff_input(10.0,0.1,gen4.wheel_rad * 1.5)

    if t == int(32/t_step):
        gen2.biases[:] = gen2.b_q_off
        gen2.diff_input(5.0,0.0,gen2.wheel_rad*1.5)
        gen3.diff_input(5.0,0.0,gen3.wheel_rad*1.5)

for i in range(6):
    axs[i].plot(x,y[i].T)
    axs[i].set_title(titles[i])
    axs[i].legend(['0','1','2','3'])

plt.figure(2)
plt.plot(px,py)
plt.show(block=True)




