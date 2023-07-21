import math
import can
import cantools
import time
import platform

db = cantools.database.load_file("odrive-cansimple.dbc")

if platform.system() == 'Linux':
    bus = can.Bus("can0",bustype="socketcan")
else:
    bus = None

axisID = 0xC

CAL = False # calibrate before getting positions

heartbeatID = axisID <<5 | db.get_message_by_name('Heartbeat').frame_id

if CAL:

    print("send axis calib 0x03 to "+str(axisID))
    
    msg = db.get_message_by_name('Set_Axis_State')
    data = msg.encode({'Axis_Requested_State':0x03})
    
    # arbitration upper
    msg = can.Message(arbitration_id=msg.frame_id | axisID << 5, is_extended_id=False, data=data)
    print(db.decode_message('Set_Axis_State',msg.data))
    print(msg)
    
    try:
        bus.send(msg)
        print("msg sent on {}".format(bus.channel_info))
    except can.CanError:
        print("msg failed")
    
while False:
    msg = bus.recv()
    # check if arb ID is from correct axis or'ed with heartbeat message ID
    if msg.arbitration_id == ((axisID << 5) | db.get_message_by_name('Heartbeat').frame_id):
        # decode heartbeat message and extract axis state bits
        current_state = db.decode_message('Heartbeat',msg.data)['Axis_State']
        if current_state == 0x1:
            print("idling")
            break
    
# can also iterate over bus to get messsages
for msg in bus:
    if msg.arbitration_id == heartbeatID:
        errCode = db.decode_message('Heartbeat',msg.data)['Axis_Error']
        if errCode == 0x00:
            print("no error")
        else:
            print("AXIS ERROR: ",str(hex(errCode)))
        break
    
reqAID = db.get_message_by_name("Get_Encoder_Estimates").frame_id | axisID << 5
request_pos = can.Message(arbitration_id = reqAID, is_extended_id=False, dlc=8, is_remote_frame=True)  
     
for i in range(100):
    t0 = time.monotonic_ns()
    bus.send(request_pos)
    t1 = time.monotonic_ns()
    for msg in bus:
        if msg.arbitration_id == reqAID:
            # decode heartbeat message and extract axis state bits
            pos = db.decode_message('Get_Encoder_Estimates',msg.data)['Pos_Estimate']
            break
    t3 =time.monotonic_ns()
    print("%d %d"%(t1-t0,t3-t1))
    print(pos)


    
bus.shutdown()