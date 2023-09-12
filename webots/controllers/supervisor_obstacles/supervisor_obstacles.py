from controller import Supervisor, Keyboard, Field
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
from datetime import datetime
import time

tracker = Supervisor()

time_step = int(tracker.getBasicTimeStep())

pdata = []
rdata = []
tdata = []

test_title = 'kura_climb'
test_names = ['1_block','2_pipe','3_rocks','4_piperock']

block_pos_T = [(0.85,0.0,0.03), (0.85,0.0,-0.2), (0.85,0.0,-0.2), (0.85,0.0,-0.2)]
pipeA_pos_T = [(0.69,0,-0.2), (0.69,0,0.03), (0.69,0,-0.2), (0.41,0,0.02)]
rocks_pos_T = [(0,0,-0.4),(0,0,-0.4),(0,0,0),(0,0,0)]

# get obstacles nodes
block = tracker.getFromDef('block')
pipeA = tracker.getFromDef('pipeA')
rocks = tracker.getFromDef('rocks')

block_pos = block.getField('translation')
pipeA_pos = pipeA.getField('translation')
rocks_pos = rocks.getField('translation')

# get robot and keyboard?
robot_node = tracker.getFromDef('whegs')
keyboard = Keyboard()
keyboard.enable(int(time_step))

# helper functions
def csv_write(data,name):
    with open('../../data/'+name+'.csv','w') as f:
        w = csv.writer(f)
        
        for i in range(len(data[0])):

            row = []
            for k in range(len(data)):  
                row = row + data[k][i]

            w.writerow(row)

def write_setting(settings):
    # list of: next run # and next test #
    with open('../../data/settings.txt','w') as f:
        ss = []
        for s in settings:
            ss.append(str(s))
        f.write(' '.join(ss))

def check_setting():
    with open('../../data/settings.txt','r') as f:
        setting = f.read().split(' ')
        si = []
        for s in setting:
            si.append(int(s))
        return(si)

# get the current test and run
run_n,test_n = check_setting()
print("current run and test: ",run_n,test_n)

# set settings for current test
block_pos.setSFVec3f(list(block_pos_T[test_n]))
pipeA_pos.setSFVec3f(list(pipeA_pos_T[test_n]))
rocks_pos.setSFVec3f(list(rocks_pos_T[test_n]))


sim_time = 0
while tracker.step(time_step) != -1:

    # start recording after zero and settling time
    if sim_time>1000:
        pos = robot_node.getPosition()
        rot = R.from_matrix(np.reshape(robot_node.getOrientation(),(3,3))).as_euler('xyz')
        
        pdata.append(list(pos))
        rdata.append(list(rot))
        tdata.append([sim_time])


    sim_time+=time_step
    # if time/1000 > 15:
    #     time_str = datetime.now().strftime("%H.%M.%S")
    #     print("writing data to: "+time_str)

    #     csv_write([tdata,pdata,rdata],time_str)

    #     break

    if sim_time > 30000:
        # exit after X seconds
        break
        
file_name = '_'.join([test_title,test_names[test_n],str(run_n),datetime.now().strftime("%H%M%S")])
print("writing data to: "+file_name)

csv_write([tdata,pdata,rdata],file_name)

run_n += 1

if run_n > 12:
    run_n = 0
    test_n += 1

if test_n == 4:
    tracker.simulationSetMode(tracker.SIMULATION_MODE_PAUSE)
else:
    print("next run and test: ",run_n,test_n)
    write_setting([run_n,test_n])

    # reset everything
    time.sleep(1.0)
    tracker.worldReload()






