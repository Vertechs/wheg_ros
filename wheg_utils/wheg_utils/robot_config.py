from whegs_4bar import WhegFourBar

# storing robot configs as python classes instead of in ini file

class RobotDefinition:
    def __init__(self,wheels):
        self.num_wheels = num_wheels
        
        # Hardware variables
        self.drive_sn = ['']*self.num_wheels
        
        # Mechanical variables
        self.modules = []
        for i in range(self.num_wheels)
            self.modules.append(WheelDefinition())
        
        
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

        
def get_config_A():
    # definin current prototype configuration
    robcfg = RobotDefinition(4)
    
    # odrive serial numbers in order
    robcfg.drive_sn = ['208839824D4D', '205839844D4D', '209039854D4D', '205239824D4D']
    
    robcfg.modules[0].outer_stages = [(60,20),(60,20)]
    robcfg.modules[0].inner_stages = [(60,20),(60,20)]
    robcfg.modules[0].wheel_stages = [(1,1)]
    robcfg.modules[0].set_ratios()
    robcfg.modules[0].n_arc = 5
    
    robcfg.modules[0].outer_stages = [(40,14),(40,14)]
    robcfg.modules[0].inner_stages = [(40,14),(40,14)]
    robcfg.modules[0].wheel_stages = [(1,1)]
    robcfg.modules[0].set_ratios()
    robcfg.modules[0].n_arc = 5
    
    robcfg.modules[0].outer_stages = [(40,14),(40,14)]
    robcfg.modules[0].inner_stages = [(40,14),(40,14)]
    robcfg.modules[0].wheel_stages = [(24+82,24)] # S / (R+S) for planetary
    robcfg.modules[0].set_ratios()
    robcfg.modules[0].n_arc = 5
    
    robcfg.modules[0].outer_stages = [(40,14),(40,14)]
    robcfg.modules[0].inner_stages = [(40,14),(40,14)]
    robcfg.modules[0].wheel_stages = [(32+80,32)] # S / (R+S) for planetary
    robcfg.modules[0].set_ratios()
    robcfg.modules[0].n_arc = 5
    
    return robcfg