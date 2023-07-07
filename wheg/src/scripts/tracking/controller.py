import odrive
import numpy as np
import time
import odrive.enums as onum
import platform

# input mode passthrough
# velocity control, maybe feedforward torque when known TODO

## Mechanical config
# move this to config file when in node
SerialNumbers = ["208839824D4D","205839844D4D"]
WheelGearRatios = []
DriveGearRatios = np.array(
                            [ [(60/20)*(60/20), (60/20)*(60/20)],
                              [(40/14)*(40/14), (40/14)*(40/14)],
                              [(40/14)*(40/14), (40/14)*(40/14)],
                              [(40/14)*(40/14), (40/14)*(40/14)] ] )
DriveGearRatios = DriveGearRatios.flatten()

## Controller config
# xfrm axle phases to wheel phase and extension
# for one axis M = [1 0; -1 1] or [.5 .5; -1 1]
# should be a better way to construct this
M1 = np.array([[1,0,0,0,0,0,0,0],
               [0,0,1,0,0,0,0,0],
               [0,0,0,0,1,0,0,0],
               [0,0,0,0,0,0,1,0]]) # phi units in radians, xfrm to radians

M2 = np.array([[-1,1,0,0,0,0,0,0],
               [0,0,-1,1,0,0,0,0],
               [0,0,0,0,-1,1,0,0],
               [0,0,0,0,0,0,-1,1]]) # "^", xfrm to extend amount 0->1

# would be non-linear state dependant if using mm of extension TODO
FULL_DEPLOY_PHASE_DIFF = np.deg2rad(60.3)

M = np.vstack([M1,M2*FULL_DEPLOY_PHASE_DIFF])
M_INV = np.linalg.inv(M)

# proportional gain matrix,
K_deploy = 20 # extensions/s per extension..?
K_phase = 20 # rad/s per rad of error
K1 = np.eye(4)*K_deploy # phase position gains
K2 = np.diag([]) # deploy amount gains assume

# get all odrives connected to system
if platform.system() == 'Linux':
    ## connect to odrives in order of serial numbers list
    drives = []
    for SN in SerialNumbers:
        drv = odrive.find_any(serial_number=SN,timeout=10)
        drives.append(drv)
        print("Odrive ",SN,":")
        odrive.utils.dump_errors(drv)

else: # assume running in test environment without drives connected
    from wheg.src.classes.dummy_odrive import odriveDummy
    drives = []
    for SN in SerialNumbers:
        drv = odriveDummy(SN)
        drives.append(drv)

# set up list of motor axes
axes = []
for drive in drives:
    axes.append(drive.axis0)
    axes.append(drive.axis1)

N_AX = len(axes)

# setup axis control and input modes
for axis in axes:
    axis.requested_state = onum.AxisState.IDLE
    axis.controller.config.control_mode = onum.ControlMode.VELOCITY_CONTROL
    axis.controller.config.input_mode = onum.InputMode.PASSTHROUGH

cmd = input("Reset to zero position (y/n) ")
if cmd == 'y':
    for axis in axes:
        axis.encoder.set_linear_count(0)

for axis in axes:
    axis.controller.input_vel = 0.0
    axis.requested_state = onum.AxisState.CLOSED_LOOP_CONTROL

# record delta time between control loops (last 10)
deltaT = np.zeros(10)
lastT = time.monotonic()
go = False
## enter main loop (to get input)
while True:
    ## get commanded position in wheel phase and extension
    # should get from topic when running as node
    # message 8x float, phases 0-3, extensions 4-8, phase in radians, deployment in [0,1]
    cmd = input("command or phase1...phaseN ext1...extN: ")

    # quit key
    if cmd == 'q':
        go = False
        break

    try:
        msg = list(map(float,cmd.split()))
        go = True
    except:
        print("invalid")
        go = False
        continue

    phase_command = np.array(msg[0:4])
    deploy_command = np.array(msg[4:8])

    num = 100 # static filter loop number for now
    # triangular position filtering with this weird for loop and linspace 0->1
    for step in np.linspace(0,1,num):
        phaseRef = phase_command * step
        deployRef = deploy_command * step

        # actual main loop cause i dont know how input() works..
        ctrl_iter = 0
        while go:
            deltaT[:-1] = deltaT[1:] # left shift time values
            deltaT[-1] = time.monotonic()-lastT
            lastT = time.monotonic() # should be using RosPy timing? TODO

            ## get estimated positions and convert to radians from revolutions
            motorPhiHat = np.array([ax.encoder.pos_estimate for ax in axes]) * (2*np.pi)
            phiHat = np.divide(motorPhiHat,DriveGearRatios[0:N_AX]) * (2*np.pi)

            ## get phase and extension errors, decoupled?
            ## wheel frame (phi_i1,phi_i2) is angular position of actual wheel hubs
            ## deploy frame (phase,deploy) is angular position of the wheel and "pseudo-linear" amount of extension i.e. effective radius
            # err = np.matmul(M,phiHat) # use transformation matrix
            phaseErr = np.zeros(4)
            deployErr = np.zeros(4)
            for i in range(len(drives)):
                phaseErr[i] = phiHat[2*i] - phaseRef[i] # assume outer phase is wheel phase (not true)
                # extending when inner phase > outer_phase
                deployErr[i] = phiHat[2*i+1] - phiHat[2*i] - deployRef[i] # inner - outer - reference

            ## get commanded phase/deploy "velocities"
            # phdp_response = np.matmul(K,err) # use gain matrix
            deltaDeployU = deployErr * K_deploy # in linear/s
            deltaPhaseU = phaseErr * K_phase # in rad/s

            ## transform back to wheel hub angular velocities
            # ph_U = np.matmul( M_INV, phdp_response) # use inverse transformation matrix
            vel_phi = np.zeros(8)
            for i in range(len(drives)):
                vel_phi[2*i]   = deltaPhaseU[i] + -deltaDeployU[i] / FULL_DEPLOY_PHASE_DIFF # outer phase
                vel_phi[2*i+1] = deltaPhaseU[i] + deltaDeployU[i] / FULL_DEPLOY_PHASE_DIFF  # inner phase

            # send velocity command to drives
            for i in range(N_AX):
                axes[i].controller.input_vel = vel_phi[i]

            if vel_phi < 1e-2 or ctrl_iter > 1e3: # arbitrary exit to get more inputs, should switch to position hold
                go = False
            # check time and exit if been too long TODO

            ctrl_iter += 1



## always set axes to idle when exiting
print("idling")
for ax in axes:
    ax.requested_state = onum.AxisState.IDLE