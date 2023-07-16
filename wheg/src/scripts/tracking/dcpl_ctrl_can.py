


## implementing "decoupled" proportional controller with velocity control signal
## using native interface for setup, state commands, etc
## using CAN for controller inputs and outputs TODO *raise encoder sending frequency

## controller and odrive comms run in single node per drive, or all drives??
## node publishes wheel states and subscribes to wheel frame position commands (wheel pos, ext 0->1)
## should also include torque feed forward from inverse kinematics

## TODO all wheels on one node and IK above, or IK and comms in node per wheel?
# probably should be node per wheel with seperate IK nodes, layer controls