import numpy as np

import wheg_utils.central_pattern_generators as cpg
import matplotlib.pyplot as plt


gen = cpg.GeneratorMatsuoka(1)

max_iter = int(1e4)
t_step = 0.01

y = np.zeros((max_iter,4))
x = np.linspace(0,max_iter*t_step,max_iter)

gen.time_constants[0,0] = 0.04
gen.time_constants[1,0] = 0.4
gen.set_input(0,1.0)

gen.w_own[:,0] = np.ones(2) * 2.0
gen.w_mut[:,0] = np.ones(2) * 2.5
gen.w_btwn = gen.w_btwn * -1.0

gen.set_state(0,0.011,0.0081,0.0022,0.0057)

for t in range(max_iter):
    gen.euler_update(t_step)
    y[t,:] = gen.u[0,:]

print(np.shape(y))
plt.plot(x,y)
plt.legend(['a','b','c','d'])
plt.show()




