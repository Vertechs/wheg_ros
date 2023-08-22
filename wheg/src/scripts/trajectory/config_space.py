import numpy as np
from wheg_utils.four_bar_wheg_circ import WhegFourBar
import matplotlib.pyplot as plt

parameters = [5,15.0,45.521,
              30.0,65.0,62.337,
              8.0,np.deg2rad(58.7)]

# x = 107.7
# y = -45.4 - 8

wheel = WhegFourBar(parameters, "wheg")

num = 200

phi1 = np.linspace(-np.pi,np.pi,num)
phi2 = np.linspace(-np.pi,np.pi,num)

space = np.zeros((num,num,3),dtype=np.uint8)
colors=np.array([[255,100,100],[100,255,100],[100,100,255],[255,0,255],[255,255,0]])


Py = np.zeros(5)

for i in range(num):
    for j in range(num):
        try:
            for n in range(5):
                P,_ = wheel.calc_FK(phi1[i]+wheel.stepAngle*n,phi2[j]+wheel.stepAngle*n)
                Py[n] = P[1]
            low = np.min(Py)
            for n in range(5):
                if Py[n] == low:
                    space[i,j] = colors[n]
        except:
            pass


print(np.max(space))
plt.imshow(space)
plt.show()
