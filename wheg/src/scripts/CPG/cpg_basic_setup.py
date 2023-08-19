import numpy as np
import matplotlib.pyplot as plt
from wheg_utils.central_pattern_generators import GeneratorKuramoto

# # Tkinter setup
# root = tk.Tk()
# root.geometry("900x1200")
#
# canvas = tk.Canvas(root, width=900, height=1200)
# canvas.create_oval(0, 0, 10, 10, fill="green")
#
# canvas.pack()
# root.mainloop()

# mpl setup
fig = plt.figure()
ax = fig.add_subplot(111)
plt.ion()

# CPG object setup
CPG = GeneratorKuramoto(4)
CPG.weights_own = np.zeros((4, 4))
CPG.gain_amp = 2.0

CPG.target_amps = np.array([0.5,0.5,0.5,0.5])
CPG.target_offs = np.zeros(4)

# Oscillator phase biases and weights
# CPG.biases = np.array([[0,0,0,1],
#                        [1,0,0,0],
#                        [0,1,0,0],
#                        [0,0,1,0]]) * (np.pi/2)
# CPG.weights_own = np.array([[0, 0, 0, 1],
#                             [1,0,0,0],
#                             [0,1,0,0],
#                             [0,0,1,0]]) * 0.2

CPG.weights = np.ones((4, 4)) - np.eye(4)
CPG.biases = np.array([[0 ,2 ,1 ,3],
                        [-2,0 ,-1,1],
                        [-1,1 ,0 ,2],
                        [-1,-1,-2,0]]) * (np.pi/2)

T = 0.01
Tstop = 40
max_iter = round(Tstop/T)
lines = np.zeros((max_iter,4))
time = np.zeros(max_iter)
CPG.perturbation([1, 1, 0])
for i in range(max_iter):
    CPG.euler_update(T)
    outputs = CPG.graph_output()
    time[i] = i*T
    lines[i,:] = outputs
    # CPG.pertubation([0.001,0.0,0.0])
    # if np.abs(i - max_iter*0.6)>10:
    #     CPG.pertubation([0.001,0.001,0])

print(np.shape(lines))
ax.plot(time,lines)
ax.legend(["A","B","C","D"])

plt.show(block=True)