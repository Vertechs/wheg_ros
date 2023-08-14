## Empty classes to cover most common attributes
## TODO: should probably just pickle a running instance of odrv class

class configDummy():
    def __init__(self):
        # cover config for any classes
        self.input_mode = 0
        self.control_mode = 0
        self.pos_gain = 0
        self.vel_gain = 0
        self.vel_integrator_gain = 0

class encoderDummy():
    def __init__(self):
        self.pos_estimate = 0
        self.config = configDummy()
    def set_linear_count(self,number):
        self.pos_estimate = number

class controllerDummy():
    def __init__(self):
        self.input_pos = 0
        self.input_vel = 0
        self.input_torque = 0
        self.config = configDummy()

class axisDummy():
    def __init__(self):
        self.encoder = encoderDummy()
        self.current_state = 0
        self.requested_state = 0
        self.config = configDummy()
        self.controller = controllerDummy()


class odriveDummy():
    def __init__(self,SN):
        self.serial_number = SN
        self.axis1 = axisDummy()
        self.axis0 = axisDummy()