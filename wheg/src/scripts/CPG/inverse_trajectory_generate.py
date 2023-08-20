import numpy as np
from wheg_utils.four_bar_wheg_circ import WhegFourBar
import matplotlib.pyplot as plt

parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]

# x = 107.7
# y = -45.4 - 8

wheel = WhegFourBar(parameters, "wheg")

steps = 100
rotation = 1

x_start = 90
x_stop = -50
com_height = 30

trajectory = np.zeros((2, rotation * steps))
xy = np.zeros((2,rotation * steps))


for i in range(rotation):
    x = x_start

    y = -com_height
    for j in range(steps):

        wheel.move_IK(x, y, i % 5)

        trajectory[0, steps * i + j] = np.rad2deg(wheel.phi1)
        trajectory[1, steps * i + j] = np.rad2deg(wheel.phi2)

        P,_ = wheel.calc_FK(wheel.phi1, wheel.phi2)

        xy[:, steps * i + j] = np.array([P[0],P[1]])

        x += (x_stop - x_start)/steps
        break

np.savetxt("traj.csv", np.deg2rad(trajectory), delimiter=",")

diff = trajectory[0,:] - trajectory[1,:]

plt.figure(1)
plt.subplot(221)
plt.plot(trajectory.T)
plt.legend(["phi1","phi2"])

plt.subplot(222)
plt.plot(diff.T)

plt.subplot(223)
plt.plot(xy.T)
plt.legend(["x","y"])

plt.show(block=True)