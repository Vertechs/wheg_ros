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

print("send axis calib 0x03 to "+str(axisID))

msg = db.get_message_by_name('Set_Axis_State')
data = msg.encode({'Axis_Requested_State':0x03})

# arbitration upper
msg = can.Message(arbitration_id=msg.frame_id | axisID << 5, is_extended_id=False, data=data)
print(db.decode_message('Set_Axis_State',msg.data))
print(msg)