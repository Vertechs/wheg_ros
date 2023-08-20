import numpy as np

import wheg_utils.central_pattern_generators as cpg
import matplotlib.pyplot as plt


## Mastuoka oscillator setup
gen1 = cpg.GeneratorMatsuokaNet(4)

gen1.time_constants[0, :] = 0.04
gen1.time_constants[1, :] = 0.4
gen1.set_input(0, 1.0)

gen1.w_own[:, :] = np.ones((2,4)) * 2.0
gen1.w_mut[:, :] = np.ones((2,4)) * 2.5

gen1.set_state_all(np.array([[-0.54,0.38,0.21,-0.58],
                             [0.16,0.11,0.22,0.10],
                             [0.38,-0.54,-0.58,0.21],
                             [0.11,0.16,0.10,0.22]]).T)
# gen1.set_state_single(0,0.0111,0.002,0.0081,0.0057)


## Kuramoto oscillator setup
gen2 = cpg.GeneratorKuramoto(4)

gen2.random_state = np.random.RandomState(23)

gen2.gain_amp = 2.0

gen2.target_amps = np.array([0.5, 0.5, 0.5, 0.5])
gen2.target_offs = np.zeros(4)

gen2.weights = np.ones((4, 4)) - np.eye(4)
gen2.biases = np.array([[0 ,2 ,1 ,3],
                        [-2,0 ,-1,1],
                        [-1,1 ,0 ,2],
                        [-1,-1,-2,0]]) * (np.pi/2)

gen2.perturbation(0,[.1, .1, .1])


## Modified hopf oscillators
gen3 = cpg.GeneratorHopfMod(4)

gen3.freq = np.ones((1,gen3.N)) * 5
gen3.weights_converge = np.vstack([np.ones((1,4))*5,np.ones((1,4))*50])
gen3.amplitudes = np.ones((1,gen3.N))

gen3.weights_inter = np.array([[0,-1,1,-1],
                               [-1,0,-1,1],
                               [-1,1,0,-1],
                               [1,-1,-1,0]])

for i in range(4):
    gen3.set_state(i,[0.25*i,0.2*i])


## Van der pol coupled oscillators
gen4 = cpg.GeneratorVdpNet(4)

A = np.array([1.5,2.0,20.0])[np.newaxis]
gen4.para = np.tile(A.T,gen4.N)

gen3.weights = np.array([[0,-1,1,-1],
                       [-1,0,-1,1],
                       [-1,1,0,-1],
                       [1,-1,-1,0]]) * 0.2

for i in range(4):
    gen4.set_state(i,0.1*i,0.2*i)


## Matsuoka non-coupled oscillators
gen5 = cpg.GeneratorMatsuokaSingle(1)

gen5.time_constants[0, 0] = 0.04
gen5.time_constants[1, 0] = 0.4

gen5.set_input(0, 1.0)
gen5.set_state(0, [0.011, 0.0022, 0.0081, 0.0057])

gen5.weights_own[:, 0] = np.ones(2) * 2.0
gen5.weights_mut[:, 0] = np.ones(2) * 2.5


## Generating and graphing CPG outputs
t_max = 2.5
t_step = 0.001
max_iter = int(t_max/t_step)

y1 = np.zeros((max_iter,4))
y2 = np.zeros((max_iter,4))
y3 = np.zeros((max_iter,4))
y4 = np.zeros((max_iter,4))
x = np.linspace(0,max_iter*t_step,max_iter)

fig = plt.figure(figsize=(20,8))
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)
plt.ion()

for t in range(max_iter):
    gen1.euler_update(t_step)
    gen2.euler_update(t_step)
    gen3.euler_update(t_step)
    gen4.euler_update(t_step)

    y1[t,:] = gen1.wheel_output()
    y2[t,:] = gen2.graph_output()
    y3[t,:] = gen3.graph_output()
    y4[t,:] = gen4.graph_output()


ax1.plot(x,y1)
ax1.legend(['a','b','c','d'])
ax1.set_title("mat mult")

ax2.plot(x,y2)
ax2.legend(['a','b','c','d'])
ax2.set_title("kura")

ax3.plot(x,y3)
ax3.legend(['a','b','c','d'])
ax3.set_title("mod hopf")

ax4.plot(x,y4)
ax4.legend(['a','b','c','d'])
ax4.set_title("van der pol")

plt.show(block=True)




