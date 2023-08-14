from wheg_utils.whegs_4bar import WhegFourBar

# storing robot configs as python classes instead of in ROS params

# wheel module numbering is L->R, T->B 
##   (0) O==||==O (1)
##          ||
##          ||
##   (2) O==||==O (3)

# calculated height to radius ratio at maximum extensions for give number of arcs
# see: https://www.desmos.com/calculator/xjnwwlqbdg
# may need to be adjusted to compensate for wheel geometry
EXT_RATIO = {3:1.366,4:1.707,5:1.760,6:1.732}
EXT_DIFF = {3:1.366,4:0.707,5:0.415,6:0.268}


class RobotDefinition:
    def __init__(self,num_wheels):
        self.num_wheels = num_wheels
        
        ## Set variables ##
        
        # Odrive serial numbers for USB comms
        self.drive_sn = [''] * self.num_wheels
        
        # Odrive axis CAN IDs
        self.axis_ids = [0x00] * self.num_wheels * 2
        
        # set up module objects for each wheel/drive
        self.modules = []
        for i in range(self.num_wheels):
            self.modules.append(WheelDefinition())
            
        # Wheel base length and width (mm) and offset (x,y,z)(mm) from robot center of mass
        self.wheel_base_offset = (0.0,0.0,0.0)
        self.wheel_base_length = 100
        self.wheel_base_width  = 100
        
        ## Calculated variables ##        
    
        # wheel center positions (x,y,z)(mm) relative to robot center of mass
        self.centers = [(0.0,0.0,0.0)] * self.num_wheels
        
        
    # set wheel center positions assuming a rectangular wheel base 
    def set_centers(self):
        for i in range(self.num_wheels):
            # Odd numbered wheels are on the right (y-)
            # wheels >2 are on back (x-)
            dx,dy=1,1
            if i%2 != 0: dy = -1
            if i>2: dx = -1
            
            x = dx * self.wheel_base_length / 2
            y = dy * self.wheel_base_width / 2
            o = self.wheel_base_offset
            
            self.centers[i] = (x+o[0],y+o[1],o[2])
        
        
# wheel gear ratios
class WheelDefinition:
    def __init__(self):
        self.outer_stages = [] # list of tuples for stage ratios
        self.inner_stages = []
        self.wheel_stages = []
        
        # hub to wheel so all >1
        self.out_ratio = 1.0
        self.inr_ratio = 1.0
        self.whl_ratio = 1.0
        
        self.n_arc = 5
        
        self.radius = 70
        
    def set_ratios(self):
        x = 1.0
        for stage in self.outer_stages:
            x = x*(stage[0]/stage[1])
        self.out_ratio = x
        
        x = 1.0
        for stage in self.inner_stages:
            x = x*(stage[0]/stage[1])
        self.inner_ratio = x
        
        x = 1.0
        for stage in self.wheel_stages:
            x = x*(stage[0]/stage[1])
        self.whl_ratio = x
        
        # not a gear ratio but still needs to be calculated
        try:
            self.ext_rad_ratio = EXT_RATIO[self.n_arc]
            self.ext_step_ratio = EXT_DIFF[self.n_arc]
        except KeyError:
            print("Wheel has invalid number of legs/arcs")

        
def get_config_A():
    # definin current prototype configuration
    robcfg = RobotDefinition(4)
    
    # odrive serial numbers in order
    robcfg.drive_sn = ['208839824D4D', '205839844D4D', '209039854D4D', '205239824D4D']
    
    robcfg.axis_ids = [a for a in range(0x0A,0x12)]
    
    robcfg.wheel_base_length = 12.2 * 25.4
    robcfg.wheel_base_width = 20.2 * 25.4
    robcfg.wheel_base_offset = (0,0,1.25*25.4)
    robcfg.set_centers()
    
    robcfg.modules[0].outer_stages = [(60,20),(60,20)]
    robcfg.modules[0].inner_stages = [(60,20),(60,20)]
    robcfg.modules[0].wheel_stages = [(1,1)]
    robcfg.modules[0].set_ratios()
    robcfg.modules[0].n_arc = 5
    
    robcfg.modules[1].outer_stages = [(40,14),(40,20)]
    robcfg.modules[1].inner_stages = [(40,14),(40,20)]
    robcfg.modules[1].wheel_stages = [(1,1)]
    robcfg.modules[1].set_ratios()
    robcfg.modules[1].n_arc = 5
    
    robcfg.modules[2].outer_stages = [(40,14),(40,20)]
    robcfg.modules[2].inner_stages = [(40,14),(40,20)]
    robcfg.modules[2].wheel_stages = [(24+82,24)] # S / (R+S) for planetary
    robcfg.modules[2].set_ratios()
    robcfg.modules[2].n_arc = 5
    
    robcfg.modules[3].outer_stages = [(40,14),(40,14)]
    robcfg.modules[3].inner_stages = [(40,14),(40,14)]
    robcfg.modules[3].wheel_stages = [(32+80,32)] # S / (R+S) for planetary
    robcfg.modules[3].set_ratios()
    robcfg.modules[3].n_arc = 5
    
    return robcfg