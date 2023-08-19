import numpy as np

import wheg_utils.central_pattern_generators as cpg
import matplotlib.pyplot as plt



# differential drive math for 4 wheels
A_ = np.array([[1,-1],[-1,-1],[1,-1],[-1,-1]])
def diff_drive(v,w,h,rad,dist):
    A = (1/rad) * np.matmul(A_,[np.array([[1,0],[0,dist/2]])])
    vels = np.matmul(A,np.array([v,w]))
    # extension as ratio of radius
    exts = np.ones(4)*(h/rad-1)
    return vels, exts


## Mastuoka oscillator setup
gen1 = cpg.GeneratorMatsuoka(1)

gen1.time_constants[0, 0] = 0.04
gen1.time_constants[1, 0] = 0.4
gen1.set_input(0, 1.0)

gen1.w_own[:, 0] = np.ones(2) * 2.0
gen1.w_mut[:, 0] = np.ones(2) * 2.5
gen1.w_btwn = gen1.w_btwn * -1.0

gen1.set_state(0, 0.011, 0.0022, 0.0081, 0.0057)


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

gen2.perturbation([.01, .01, .01])


## Matsuoka separate oscillators
gen3 = cpg.GeneratorMatsuokaSingle(1)

gen3.time_constants[0, 0] = 0.04
gen3.time_constants[1, 0] = 0.4

gen3.set_input(0, 1.0)
gen3.set_state(0, [0.011, 0.0022, 0.0081, 0.0057])

gen3.weights_own[:, 0] = np.ones(2) * 2.0
gen3.weights_mut[:, 0] = np.ones(2) * 2.5


## Generating and graphing CPG outputs
t_max = 10
t_step = 0.01
max_iter = int(t_max/t_step)

y1 = np.zeros((max_iter,4))
y2 = np.zeros((max_iter,4))
y3 = np.zeros((max_iter,5))
x = np.linspace(0,max_iter*t_step,max_iter)

fig = plt.figure()
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
plt.ion()

for t in range(max_iter):
    #gen1.euler_update(t_step)
    gen2.euler_update(t_step)
    gen3.euler_update(t_step)
    y1[t,:] = np.hstack((gen1.u[:,0],gen1.v[:,0]))
    y2[t,:] = gen2.graph_output()
    y3[t,0:4] = gen3.state[:,0]
    y3[t,5] = gen3.graph_output()


ax1.plot(x,y1)
ax1.legend(['u1','u2','v1','v2'])
ax1.set_title("mat mult")

ax2.plot(x,y2)
ax2.legend(['a','b','c','d'])
ax2.set_title("kura")

ax3.plot(x,y3)
ax3.legend(['u1','u2','v1','v2'])
ax3.set_title("mat sing")

plt.show(block=True)




