import numpy as np
from wheg_utils.four_bar_wheg_circ import WhegFourBar
import matplotlib.pyplot as plt

parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]

# x = 107.7
# y = -45.4 - 8

wheel = WhegFourBar(parameters, "wheg")

plot = True

iters = 101
steps = 1

com_height = 80
# x position at switch will always be when leg is 1/2*1/N rotations behind
x_start = np.sin(2*np.pi/5) * com_height
x_stop = -x_start

trajectory = np.zeros((2, steps * iters))
xy = np.zeros((2, steps * iters))
torques = np.zeros((2,steps*iters))



plt.ion()
color = ['ob', 'ob', 'og', '*k', 'xb']
if plot:
    plt.figure(1)
    plt.subplot(111)
    plt.show()


x = x_start
y = -com_height
contact = False

for S in range(steps):
    if not contact:
        x = x_start
    contact = False

    for j in range(iters):

        wheel.move_IK(x, y, S % 5)

        trajectory[0, iters * S + j] = np.rad2deg(wheel.phi1)
        trajectory[1, iters * S + j] = np.rad2deg(wheel.phi2)

        torques[:, iters*S+j] = np.array(wheel.calc_torques(0,10))

        # get phases for current and one arc behind current
        pi1 = wheel.phi1
        pi2 = wheel.phi2
        pii1 = wheel.phi1  + wheel.stepAngle
        pii2 = wheel.phi2  + wheel.stepAngle


        P2,_ = wheel.calc_FK(pi1, pi2)
        P,_ = wheel.calc_FK(pii1,pii2)
        xy[:, iters * S + j] = np.array([P2[0], P2[1]])

        # check if trailing arc made contact with ground
        # if P[1] <= y:
        #     # set new x value, break out of loop to increment i
        #     x = P[0]
        #     contact = True
        #     break

        x += (x_stop - x_start) / iters

        if plot:
            A = _[0]
            B = _[1]
            plt.clf()
            for p in range(5):
                plt.plot(wheel.get_points()[p,0],wheel.get_points()[p,1],color[p])

            plt.plot(A[0], A[1],'*y')
            plt.plot(P[0], P[1], 'xy')
            plt.plot(B[0], B[1], '*y')
            plt.legend(['A', 'B', 'C', 'D', 'P', 'P(FK)'])
            plt.xlim([-100, 100])
            plt.ylim([-100, 100])
            plt.draw()
            plt.pause(0.01)



np.savetxt("traj.csv", np.deg2rad(trajectory), delimiter=",")

diff = trajectory[0,:] - trajectory[1,:]

plt.figure(2)
plt.subplot(221)
plt.plot(trajectory.T)
plt.legend(["phi1","phi2"])

plt.subplot(222)
plt.plot(diff.T)

plt.subplot(223)
plt.plot(xy.T)
plt.legend(["x","y"])

plt.subplot(224)
plt.plot(torques.T)
plt.legend(['T1','T2'])

plt.show(block=True)