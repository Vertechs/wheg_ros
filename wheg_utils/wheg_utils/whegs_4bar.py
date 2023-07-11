import numpy as np
import math

# Four bar mechanism with separately driven inner and outer hub
# assuming crossed links in closed position
class WhegFourBar:
    def __init__(self, param, name="wheg"):
        self.name = name

        # mechanical parameters
        self.arcN       = int(param[0])
        self.arcPivotRadius = param[1] # from arc/hub pivot to arc/link pivot
        self.linkLength     = param[2]
        self.inHubRadius    = param[3]
        self.outHubRadius   = param[4]
        self.arcLength      = param[5] # from arc/hub pivot to center of tip radius
        self.endRadius      = param[6] # assume circular contact for tip of arc
        self.pivotTheta     = param[7] # (rad) angle around arc/hub pivot between line to tip and line to link pivot

        # calc extra parameters
        self.arcRadius = self.outHubRadius
        self.angle = 2*np.pi/self.arcN

        # dynamic variables
        self.phi1 = 0.0 # phase of outer hub
        self.phi2 = 0.0 # phase of inner hub
        self.thA = self.calc_thA(self.phi1,self.phi2) # angle between DA and AB

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

        return ph1,ph2,thA

    def single_move_IK(self,position,arc_n):
        ph1,ph2,thA = self.calc_single_IK(position)
        if math.isnan(ph1):
            return -1
        else:
            self.thA = thA
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

        thetaA = self.calc_thA(ph1,ph2)

        endx = Ax + self.arcLength * np.cos(ph1+thetaA)
        endy = Ay + self.arcLength * np.sin(ph1+thetaA)

        return endx,endy

    def update_angles(self):
        self.thA = self.calc_thA(self.phi1,self.phi2)

    def calc_thA(self,ph1,ph2):
        DA = self.outHubRadius
        DC = self.inHubRadius
        CB = self.linkLength
        AB = self.arcPivotRadius

        # lots of law of cosines
        # distance between A and C, inner and outer hub pivot points
        CA = np.sqrt( DA**2 + DC**2 - 2*DA*DC * np.cos(ph2-ph1) )
        # angle between CA line and AB (arc pivot arm)
        alpha = np.arccos( (CB**2-CA**2-AB**2)/(-2*CA*AB) )
        # angle between CA line and DA (center to outer pivot)
        beta = np.arccos( (DC**2-CA**2-DA**2)/(-2*CA*DA) )

        thA = -np.pi + alpha - beta + self.pivotTheta #thA between AD and AL, add thBA angle

        return thA
