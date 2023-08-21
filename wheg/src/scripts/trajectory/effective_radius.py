import numpy as np
from wheg_utils.four_bar_wheg_circ import WhegFourBar
import matplotlib.pyplot as plt


def perimeter(num_sides, outer_radius):
    a = 2 * outer_radius * np.sin(np.pi/num_sides)
    return a * num_sides


parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]

wheel = WhegFourBar(parameters, "wheg")

# get phase difference with no extension and max extension
wheel.move_IK(wheel.outHubRadius,0.0)
p_start = wheel.phi2 - wheel.phi1
print(wheel)
wheel.move_IK(wheel.outHubRadius+wheel.arcLength,0.0)
p_stop = wheel.phi2 - wheel.phi1
print(wheel)

x = np.linspace(p_start,p_stop,100)
y = np.zeros((3,len(x)))

for i in range(len(x)):
    p = x[i]
    try:
        P,_ = wheel.calc_FK(0.0,p)
    except:
        print('ee')
        break

    rad = np.linalg.norm(P)
    per = perimeter(5,rad)
    e_rad = per/(2*np.pi)
    y[:,i] = [rad,e_rad,per]


plt.figure(1)
plt.plot(x,y[0:2,:].T)
plt.legend(['rad','erd'])
plt.show()