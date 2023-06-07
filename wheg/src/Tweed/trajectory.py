import numpy as np
from whegs import WhegFourBar
import matplotlib.pyplot as plt


parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]

# x = 107.7
# y = -45.4 - 8

whl1 = WhegFourBar(parameters)

steps = 100
rotation = 5

x_start = 110
com_height = 50

trajectory = np.zeros((2, rotation * steps))

for i in range(rotation):
    x = x_start
    y = -com_height
    for j in range(steps):

        whl1.single_move_IK([x, y], i%5)

        trajectory[0, steps * i + j] = np.rad2deg(whl1.phi1)
        trajectory[1, steps * i + j] = np.rad2deg(whl1.phi2)

        ax,ay = whl1.calc_single_FK(whl1.phi1, whl1.phi2)

        x += -2 * x_start/steps

plt.figure(1)
plt.plot(trajectory.T)
plt.legend(["phi1","phi2"])

plt.show(block=True)