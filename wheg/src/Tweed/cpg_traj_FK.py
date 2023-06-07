import numpy as np
from whegs import WhegFourBar
import matplotlib.pyplot as plt
from CPG.CPG_FIC import CPG_FIC

parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]
whl = []
names = ["1","2","3","4"]
for i in range(4):
    whl.append(WhegFourBar(parameters,names[i]))

gen1 = CPG_FIC(4)

# gen1.weights = np.array([[0,0,0,0],
#                        [1,0,0,0],
#                        [1,0,0,0],
#                        [1,0,0,0]]) * 0.2
# gen1.biases = np.array([[0,0,0,0],
#                        [1,0,0,0],
#                        [2,0,0,0],
#                        [3,0,0,0]]) * (np.pi/4)
gen1.weights = np.array([[0,0,0,0],
                         [1,0,0,0],
                         [0,1,0,0],
                         [0,0,1,0]]) * 1.0
gen1.biases = np.array([[0,0,0,0],
                        [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0]]) * (np.pi/4)

height = 50
step_max = 100

gen1.target_amps = np.ones(4)*np.pi/4
gen1.target_offs = np.zeros(4)
gen1.own_freq = np.ones(4)*5

T = 0.01
Tstop = 20
max_iter = round(Tstop/T)

posx_out = np.zeros((4, max_iter))
posy_out = np.zeros((4, max_iter))
time = np.zeros(max_iter)
ph1_out = np.zeros((4, max_iter))
ph2_out = np.zeros((4, max_iter))

def out_callback(off,amp,phi):
    # magic IK approximating function
    ph1 = phi + np.sin(phi)
    ph2 = phi + np.sin(phi) + amp
    return ph1,ph2


gen1.pertubation([.1,.1,0])
for i in range(max_iter):
    # update CPG and get output
    output = gen1.euler_update(T,out_callback)
    ph1_out[:,i] = output[:,0]
    ph2_out[:,i] = output[:,1]
    time[i] = i*T

    k = 0
    for wheg in whl:
        # move IK with cpg output
        ph1 = output[k,0]
        ph2 = output[k,1]
        ax,ay = wheg.single_move_FK(ph1,ph2)

        posx_out[k,i] = ax
        posy_out[k,i] = ay

        k += 1

plt.subplot(221)
plt.plot(time.T,posx_out.T)
plt.legend(["1","2","3","4"])
plt.title("X position")

plt.subplot(222)
plt.plot(time.T,posy_out.T)
plt.legend(["1","2","3","4"])
plt.title("Y position")

plt.subplot(223)
plt.plot(time.T,ph1_out.T)
plt.legend(["1","2","3","4"])
plt.title("Outer Phase")

plt.subplot(224)
plt.plot(time.T,ph2_out.T)
plt.legend(["1","2","3","4"])
plt.title("Inner Phase")

plt.show(block=True)






