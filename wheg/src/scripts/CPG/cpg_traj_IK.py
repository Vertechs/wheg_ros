import numpy as np
from wheg.wheg_utils.classes.whegs import WhegFourBar
import matplotlib.pyplot as plt
from wheg.wheg_utils.classes.CPG_FIC import CPG_FIC

parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]
whl = []
names = ["1","2","3","4"]
for i in range(4):
    whl.append(WhegFourBar(parameters,names[i]))

gen1 = GeneratorFC(4)

# gen1.weights = np.array([[0,0,0,0],
#                        [1,0,0,0],
#                        [1,0,0,0],
#                        [1,0,0,0]]) * 0.2
# gen1.biases = np.array([[0,0,0,0],
#                        [1,0,0,0],
#                        [2,0,0,0],
#                        [3,0,0,0]]) * (np.pi/4)
# gen1.weights = np.array([[0,0,0,0],
#                          [1,0,0,0],
#                          [0,1,0,0],
#                          [0,0,1,0]]) * 0.2
# gen1.biases = np.array([[0,0,0,0],
#                         [1,0,0,0],
#                         [0,1,0,0],
#                         [0,0,1,0]]) * (np.pi/4)

# "walking" gate, all quarter turn off
gen1.weights_own = np.ones((4, 4)) - np.eye(4)
gen1.biases = np.array([[0 ,2 ,1 ,3],
                        [-2,0 ,-1,1],
                        [-1,1 ,0 ,2],
                        [-1,-1,-2,0]]) * (np.pi/2)

gen1.gain_amp = 2.0

height = 50
step_max = 100

gen1.target_amps = np.ones(4)*height
gen1.target_offs = np.zeros(4)

T = 0.01
Tstop = 20
max_iter = round(Tstop/T)

gen_out = np.zeros((4, max_iter))
time = np.zeros(max_iter)
ph1_out = np.zeros((8, max_iter))
ph2_out = np.zeros((8, max_iter))

def out_callback(off,amp,phi):
    # value moving from 1 to -1, jump back to 1
    phi_s = np.mod(phi,np.pi)
    stp = np.floor(phi/np.pi)
    return np.cos(phi),stp,amp


gen1.pertubation([0.1,0.1,0])
for i in range(max_iter):
    # update CPG and get output
    output = gen1.euler_update(T,out_callback)
    dis_out = output[:,0]
    stp_out = output[:,1]
    amp_out = output[:,2]
    time[i] = i*T
    gen_out[:, i] = dis_out

    gen1.pertubation([0.01, 0.01, 0.01])

    posx = amp_out
    posy = dis_out*step_max

    k = 0
    for wheg in whl:
        # move IK with cpg output
        wheg.single_move_IK((posx[k],posy[k]),stp_out[k])
        ph1_out[k,i] = wheg.phi1
        ph2_out[k,i] = wheg.phi2
        k += 1

plt.subplot(311)
plt.plot(time.T,gen_out.T)
plt.legend(["1","2","3","4"])

plt.subplot(312)
plt.plot(time.T,ph1_out.T)
plt.legend(["1","2","3","4"])

plt.subplot(313)
plt.plot(time.T,ph2_out.T)
plt.legend(["1","2","3","4"])

plt.show(block=True)






