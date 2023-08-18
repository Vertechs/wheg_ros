import numpy as np
import math

# Four bar mechanism with separately driven inner and outer hub
# assuming crossed links in closed position

# very poor drawing of labeled points in four bar math
# (D) O---------O (A)
#      \        |\
#       \       |  \
#        \      |    \
#     (C) O-----O (B)  \
#                        \
#                          O (P/L)





class WhegFourBar:
    def __init__(self, param, name="wheg"):
        self.name = name

        # mechanical parameters
        self.arcN       = int(param[0])
        self.arcPivotRadius = param[1] # from arc/hub pivot to arc/link pivot (B->A)
        self.linkLength     = param[2]
        self.inHubRadius    = param[3]
        self.outHubRadius   = param[4]
        self.arcLength      = param[5] # from arc/hub pivot to center of tip radius (A->P)
        self.endRadius      = param[6] # assume circular contact for tip of arc
        self.pivotTheta     = param[7] # (rad) angle around arc/hub pivot between line to tip and line to link pivot

        # calc extra parameters
        self.arcRadius = self.outHubRadius
        self.angle = 2*np.pi/self.arcN

        # dynamic variables
        self.phi1 = 0.0 # phase of outer hub
        self.phi2 = 0.0 # phase of inner hub
        self.thA,self.thC = self.calc_int_angles(self.phi1,self.phi2) # angle between DA and AB

    def __str__(self):
        return self.name+" ph1=%.4f ph2=%.4f"%(self.phi1,self.phi2)

    def calc_single_IK(self, position):
        # offset up by end radius, assuming floor contact
        pdy = position[1]+self.endRadius
        pdx = position[0]

        # Solve two link arm with arc and outer hub
        try:
            thA = -np.arccos((pdx**2 + pdy**2 - self.outHubRadius**2 - self.arcLength**2) /
                   (2*self.outHubRadius*self.arcLength))
        except:
            print("Position %.4f %.4f is unreachable"%(pdx,pdy))
            return float("NaN"),float("NaN"),float("NaN")
        ph1 = np.arctan2(pdy,pdx) - np.arctan2(self.arcLength*np.sin(thA),
                                                self.outHubRadius+self.arcLength*np.cos(thA))

        # get arc/link pivot location (x and y)
        Ax = self.outHubRadius * np.cos(ph1)
        Ay = self.outHubRadius * np.sin(ph1)
        thBA = thA - self.pivotTheta
        Bx = Ax + self.arcPivotRadius*np.cos(ph1+thBA)
        By = Ay + self.arcPivotRadius*np.sin(ph1+thBA)

        # Solve two link arm with inner hub and link
        try:
            thC = -np.arccos((Bx**2 + By**2 - self.inHubRadius**2 - self.linkLength**2)/
                   (2*self.inHubRadius*self.linkLength))
        except:
            print("Position %.4f %.4f is unreachable"%(pdx,pdy))
            return float("NaN"),float("NaN"),float("NaN")
        ph2 = np.arctan2(By,Bx) - np.arctan2(self.linkLength*np.sin(thC),
                                             self.inHubRadius+self.linkLength*np.cos(thC))

        return ph1,ph2,thA,thC

    def single_move_IK(self,position,arc_n):
        ph1,ph2,thA,thC = self.calc_single_IK(position)
        if math.isnan(ph1):
            return -1
        else:
            self.thA = thA
            self.thC = thC
            self.phi1 = ph1 - arc_n * self.angle # minus assuming cw rotation
            self.phi2 = ph2 - arc_n * self.angle
            return 1

    def single_move_FK(self,ph1,ph2):
        self.phi2 = ph2
        self.phi1 = ph1
        self.update_angles()

        Ax = self.outHubRadius * np.cos(ph1)
        Ay = self.outHubRadius * np.sin(ph1)

        endx = Ax + self.arcLength * np.cos(ph1+self.thA)
        endy = Ay + self.arcLength * np.sin(ph1+self.thA)

        return endx,endy

    def calc_single_FK(self, ph1, ph2):
        Ax = self.outHubRadius * np.cos(ph1)
        Ay = self.outHubRadius * np.sin(ph1)

        thetaA,thetaC = self.calc_int_angles(ph1,ph2)

        endx = Ax + self.arcLength * np.cos(ph1+thetaA)
        endy = Ay + self.arcLength * np.sin(ph1+thetaA)

        return endx,endy

    def update_angles(self):
        self.thA,self.thC = self.calc_int_angles(self.phi1,self.phi2)

    def calc_int_angles(self,ph1,ph2):
        # return angles between DA and AL and DC and CB 
        DA = self.outHubRadius
        DC = self.inHubRadius
        CB = self.linkLength
        AB = self.arcPivotRadius

        # lots of law of cosines
        # distance between A and C, inner and outer hub pivot points
        CA = np.sqrt( DA**2 + DC**2 - 2*DA*DC * np.cos(ph2-ph1) )
        # angle between CA line and AB (arc pivot arm)
        a1 = np.arccos( (CB**2-CA**2-AB**2)/(-2*CA*AB) )
        # angle between CA line and DA (center to outer pivot)
        a2 = np.arccos( (DC**2-CA**2-DA**2)/(-2*CA*DA) )
        # angle between CA line and DC 
        a3 = np.arccos( (DA**2-DC**2-CA**2)/(-2*CA*DC) )
        # angle between CA line and CB
        a4 = np.arccos( (AB**2-CB**2-CA**2)/(-2*CA*CB) )
        
        thA = -np.pi + a1 - a2 + self.pivotTheta #thA between DA and AL, add B-A-L angle
        thC = -np.pi + a3 - a4

        return thA,thC
        
        
        
    def calc_torques(self,Fx,Fz):
        # TODO check output
        # get moment about point A (arc pivot) from end point force
        Lx = self.arcLength * cos(self.ph1 + self.thA)
        Lz = self.arcLength * sin(self.ph1 + self.thA)
        M_L = -Lz * Fx + Lx * Fz
        
        # get force on link (compression negative)
        a1 = self.phi1+self.thA+self.pivotTheta
        a2 = self.phi2+self.thC
        F_l = M_L / ( (-np.sin(a1)*np.cos(a2) + np.cos(a1)*np.sin(a2)) * self.arcPivotRadius)
        
        # get inner torque from link force
        T2 = -F_l * cos(self.thC)
        
        # get outer torque from arc and link force
        F_Ax = Fx - F_l*cos(a2)
        F_Ay = Fy - F_l*sin(a2)
        T1 = (F_Ax*sin(self.phi1) + F_Ay*cos(self.phi1))*self.outHubRadius
        
        
        return T1,T2
