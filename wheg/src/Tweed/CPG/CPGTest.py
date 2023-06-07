import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from CPG_FIC import CPG_FIC

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
CPG = CPG_FIC(4)
CPG.weights = np.zeros((4,4))
CPG.gain_amp = 2.0

CPG.target_amps = np.array([0.5,0.5,0.5,0.5])
CPG.target_offs = np.zeros(4)

# Oscillator phase biases and weights
CPG.biases = np.array([[0,0,0,1],
                       [1,0,0,0],
                       [0,1,0,0],
                       [0,0,1,0]]) * (np.pi/2)
CPG.weights = np.array([[0,0,0,1],
                       [1,0,0,0],
                       [0,1,0,0],
                       [0,0,1,0]]) * 0.2

T = 0.01
Tstop = 40
max_iter = round(Tstop/T)
lines = np.zeros((4,max_iter))
time = np.zeros(max_iter)
CPG.pertubation([1,1,0])
for i in range(max_iter):
    outputs = CPG.euler_update(T,CPG.default_callback)
    time[i] = i*T
    lines[:,i] = outputs
    #CPG.pertubation([0.001,0.0,0.0])
    if np.abs(i - max_iter*0.6)>10:
        CPG.pertubation([0.001,0.001,0])

print(np.shape(lines))
ax.plot(time,lines.T)
ax.legend(["A","B","C","D"])

# scream...
plt.show(block=True)