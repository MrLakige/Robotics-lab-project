#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np
from save import *


#delete a block by its name
def delete_model(name):
	delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	return delete_model_client(model_name=name)

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
castleBlocks = ['X1-Y4-Z2', 'X1-Y3-Z2', 'X1-Y1-Z2', 'X1-Y2-Z2']

initialBlockPositions = [
		['X1-Y2-Z2', 0.280577, 0.691699, 0.870002, 0., 0., 0., 0.],
		['X1-Y1-Z2', 0.123776, 0.672390, 0.870002, 0., 0., 0., 0.],
		['X1-Y4-Z2', 0.414135, 0.725608, 0.870002, 0., 0., 0., 0.],
		['X1-Y3-Z2', 0.367994, 0.598631, 0.920000, 0., 0.7071, -0.7071, 0.]
        ]

#cleans the table in case there are blocks on it
for block in blocks:	
		delete_model(f'{block}')

clean()
                
for i in range(4):
    #Get a random lego block from all legos
    brick=initialBlockPositions[i][0]
    pos = Pose(Point(initialBlockPositions[i][1], initialBlockPositions[i][2], initialBlockPositions[i][3]), Quaternion(initialBlockPositions[i][4], initialBlockPositions[i][5], initialBlockPositions[i][6], initialBlockPositions[i][7]))
    #Call rospy spawn function to spawn objects in gazebo
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=''+str(brick), 
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        robot_namespace='/foo',
        initial_pose=pos,
        reference_frame='world')

    saveToXml(str(brick), pos, None)
