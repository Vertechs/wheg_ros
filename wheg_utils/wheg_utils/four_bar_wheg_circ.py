import numpy as np
import math
from numpy import sin,cos,tan,arctan2,arccos,pi

# Four bar mechanism with separately driven inner and outer hub
# assuming crossed links in closed position

# very poor drawing of labeled points in four bar math
# (D) O---------O (A)
#      \        | \
#       \       |   \
#     (C) O-----O     \
#                (B)    \
#                         O (P)

# This class is mainly meant for performing IK calculation, so variables can be reset often
# The actual states of the wheel should be tracked by the controller

class WhegFourBar:
    def __init__(self, param, name="wheg"):
        self.name = name

        # mechanical parameters
        self.arcN = int(param[0])
        self.arcPivotLength = param[1]  # from arc/hub pivot to arc/link pivot (B->A)
        self.linkLength = param[2]
        self.inHubRadius = param[3]
        self.outHubRadius = param[4]
        self.arcLength = param[5]  # from arc/hub pivot to center of tip radius (A->P)
        self.endRadius = param[6]  # assume circular contact for tip of arc
        self.pivotAngle = param[7]  # (rad) angle around arc/hub pivot between line to tip and line to link pivot

        # calc extra parameters
        self.arcRadius = self.outHubRadius
        self.stepAngle = 2 * pi / self.arcN

        # dynamic variables
        self.pO = 0.0  # phase of outer hub to be used in 4 bar calcs for active arc
        self.pI = 0.0  # "      " inner "  "

        self.offset = 0 # integer (ccw = +) selector for currently active arc
        self.phi1 = 0.0 # actual outer hub phase, including integer offset for active arc
        self.phi2 = 0.0 # "    " inner "   "


        # pivot points of 4 bar mechanism and end point
        # we can always add a multiple of self.angle when stepping with a different arc
        # so we can just assume these points are for the active arc mechanism
        self.A = np.zeros(2)
        self.B = np.zeros(2)
        self.C = np.zeros(2)
        self.D = np.zeros(2) # should always be (0,0) in this object
        self.P = np.zeros(2)

        self.thAP = 0.0 # angle of the A->P vector
        self.thAB = 0.0 # angle of the A->B vector
        self.thCB = 0.0 # angle of the C->B vector


    def __str__(self):
        return self.name + " ph1=%.4f ph2=%.4f" % (self.pI, self.pO)

    def circle_intersect(self, x1, y1, r1, x2, y2, r2, side):
        """
        Calculate intersection point of two circles
        :param side: +1 : return pt cw from center 1 | -1 : vice versa
        :return: -1 if no intersect, else x,y coord of intersect
        """
        # dist between circles
        dx = x2 - x1
        dy = y2 - y1
        d = np.sqrt(dx**2 + dy**2)

        # check if circles intersect
        if d > r1+r2:
            raise Exception('circles too far')
        elif d < np.abs(r1-r2):
            raise Exception('circles nested')
        elif d == 0:
            raise Exception('circles same point')

        cd = (r1**2-r2**2+d**2) / (2*d) # chord distance along center line
        ch = np.sqrt(r1**2-cd**2) # height of intersect from chord
        xm = x1+cd*(x2-x1)/d # point in middle of intersection lense
        ym = y1+cd*(y2-y1)/d

        # print(' s:',side,'dx',dx,'dy',dy,end='')

        # pick intersection point based on side
        xi = xm + side * ch * dy/d
        yi = ym - side * ch * dx/d
        return xi,yi

    def move_IK(self, px, py, n=None):
        """
        Take target position of arc endpoint and return phase values for wheel hubs
        """
        # set currently active arc if given
        if n:
            self.offset = n

        # from end point we can get point A, from those we can get outer hub (phi1) and angle of leg (thA)
        self.P[0] = px
        self.P[1] = py
        self.A[:] = self.circle_intersect(0, 0, self.outHubRadius, self.P[0], self.P[1], self.arcLength, -1)
        # print(' A:',self.A,end='')
        self.pO = arctan2(self.A[1], self.A[0]) # A-D but D is always 0,0
        self.thAP = arctan2(self.P[1]-self.A[1],self.P[0]-self.A[0])

        # from point A and thAP calculate angle and points for actuating bar
        self.thAB = self.thAP - self.pivotAngle
        self.B[:] = self.A + [cos(self.thAB) * self.arcPivotLength, sin(self.thAB) * self.arcPivotLength]
        # print(' B:',self.B,end='')
        self.C[:] = self.circle_intersect(0, 0, self.inHubRadius, self.B[0], self.B[1], self.linkLength, -1)
        # print(' C:',self.C,end='\n')
        self.thCB = arctan2(self.B[1]-self.C[1],self.B[0]-self.C[0])
        self.pI = arctan2(self.C[1],self.C[0])

        # update actual phase values
        self.phi1 = self.pO + self.stepAngle * self.offset
        self.phi2 = self.pI + self.stepAngle * self.offset


    def move_FK(self, ph1, ph2, n=None):
        """
        Take phase values of wheel hubs and return position of arc endpoint
        """
        # set currently active arc if given
        if n:
            self.offset = n

        # set active arc phases
        self.pO = ph1 - self.stepAngle * self.offset
        self.pI = ph2 - self.stepAngle * self.offset

        # get positions of A and C pivots immediately
        self.A[:] = [cos(self.pO)*self.outHubRadius,sin(self.pO)*self.outHubRadius]
        self.C[:] = [cos(self.pI)*self.inHubRadius,sin(self.pI)*self.inHubRadius]

        # get B position from A and C and link and arc link lengths
        self.B[:] = self.circle_intersect(self.A[0],self.A[1],self.arcPivotLength,self.C[0],self.C[1],self.linkLength,-1)
        self.thCB = arctan2(self.B[1]-self.C[1],self.B[0]-self.C[0])
        self.thAB = arctan2(self.B[1]-self.A[1],self.B[0]-self.A[0])
        self.thAP = self.thAB + self.pivotAngle
        self.P[:] = [self.A[0]+cos(self.thAP)*self.arcLength,self.A[1]+sin(self.thAP)*self.arcLength]

    def calc_FK(self, pO, pI):
        # calculate FK values without changing any states

        # get positions of A and C pivots directly from hub phases
        A = np.array([cos(pO)*self.outHubRadius,sin(pO)*self.outHubRadius])
        C = np.array([cos(pI)*self.inHubRadius,sin(pI)*self.inHubRadius])

        # get B position from A and C and link and arc link lengths
        B = np.array(self.circle_intersect(A[0],A[1],self.arcPivotLength,C[0],C[1],self.linkLength,-1))
        thCB = arctan2(B[1]-C[1],B[0]-C[0])
        thAB = arctan2(B[1]-A[1],B[0]-A[0])

        # get end point position from A and theta AP
        thAP = thAB + self.pivotAngle
        P = np.array([A[0]+cos(thAP)*self.arcLength,A[1]+sin(thAP)*self.arcLength])
        return P, [A,B,C,thCB,thAB,thAP]

    def calc_torques(self, Fx, Fz):
        # TODO check output
        # get moment about point A (arc pivot) from end point force
        Lx = self.P[0] - self.A[0]
        Lz = self.P[1] - self.A[1]
        M_L = -Lz * Fx + Lx * Fz

        # get force from link, tension positive
        Bx = self.B[0] - self.A[0]
        Bz = self.B[1] - self.A[1]
        cosCB = cos(self.thCB)
        sinCB = sin(self.thCB)
        print(Bx,Bz,cosCB,sinCB)
        T_L = -M_L / (Bz * cosCB - Bx * sinCB)

        # get torque on inner hub from link tension and C position
        T2 = -T_L * (self.C[0] * sinCB + self.C[1]*cosCB)

        AFz = T_L * sinCB - Fz
        AFx = T_L * cosCB - Fx
        T1 = -AFx*self.A[1] + AFz*self.A[0]

        print(M_L,T_L,AFz,AFx)

        return T1, T2

    def get_points(self):
        return np.vstack([self.A,self.B,self.C,self.D,self.P])
