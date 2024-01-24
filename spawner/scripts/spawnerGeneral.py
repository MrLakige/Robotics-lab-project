#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np

blocks = { \
		'X1-Y1-Z2': (0,(0.031,0.031,0.057)), \
		'X1-Y2-Z1': (1,(0.031,0.063,0.038)), \
		'X1-Y2-Z2': (2,(0.031,0.063,0.057)), \
		'X1-Y2-Z2-CHAMFER': (3,(0.031,0.063,0.057)), \
		'X1-Y2-Z2-TWINFILLET': (4,(0.031,0.063,0.057)), \
		'X1-Y3-Z2': (5,(0.031,0.095,0.057)), \
		'X1-Y3-Z2-FILLET': (6,(0.031,0.095,0.057)), \
		'X1-Y4-Z1': (7,(0.031,0.127,0.038)), \
		'X1-Y4-Z2': (8,(0.031,0.127,0.057)), \
		'X2-Y2-Z2': (9,(0.063,0.063,0.057)), \
		'X2-Y2-Z2-FILLET': (10,(0.063,0.063,0.057)) \
        }

blocksName = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
shufflableBlockArray = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
possibleRotations = [0., 1.57, 3.14, 4.71, 6.28]
positionsUsed = []

blockList = list(blocks.keys())
level = None
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

def getLevel():
    global level

    if len(sys.argv) != 2:
        print("Usage: rosrun levelMAnager spwaner.py N, where N stands for the task number" )
        exit(1)
    else:
        level = int(sys.argv[1])
        if(level > 0 and level <=4):
            print(level)
        else:
            print("The task number is incorrect please insert a level betwween 1-4" )
        exit(2) 

def spawnBlock(brick, pos):
    spawn_model_client(model_name=''+str(brick), 
    model_xml=open('src/levelManager/lego_models/'+brick+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')
        
         
def getPose(rotated=False):
    if (rotated==False):
        pose = Pose(Point(random.uniform(0.060266, 0.432769), random.uniform(0.565019, 0.730745), 0.863913), Quaternion(0, 0, 0, 0))
        return pose
    else:
        pose = Pose(Point(random.uniform(0.060266, 0.432769), random.uniform(0.565019, 0.730745), 0.863913), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))

def spawnLegoPerLevel():
    global level
    print(level)
    if(level == 1):
        brick=blocks[random.randint(0,10)]
        spawnBlock(brick, getPose())
    elif(level == 2):
        for i in range(11):
            notAvailable=True
            if i==0:
                pose = getPose()
                positionsUsed.append(pose)
            else:
                while notAvailable==True:
                    pose = getPose()
                    for k in range(i):
                        threshold = 0.125
                        if np.sqrt((pose.position.x-positions[k].position.x)**2+(pose.position.y-positions[k].position.y)**2) < threshold:
                            break
                        if k == i-1:
                            positionsUsed.append(pose)
                            notAvailable = False
            spawnBlock(blockList[i], pose)
    elif(level == 3):
        random.shuffle(shufflableBlockArray)
        for i in range(4):
            notAvailable=True
            if i==0:
                pose = getPose(rotated=True)
                positionsUsed.append(pose)
            else:
                while notAvailable==True:
                    pose = getPose()
                    for k in range(i):
                        threshold = 0.125
                        if np.sqrt((pose.position.x-positions[k].position.x)**2+(pose.position.y-positions[k].position.y)**2) < threshold:
                            break
                        if k == i-1:
                            positionsUsed.append(pose)
                            notAvailable = False
            spawnBlock(shufflableBlockArray[i], pose) 
    else:
        print(level)


if __name__ == '__main__':

    getLevel()
    spawnLegoPerLevel()
    rospy.init_node("levelManager")
