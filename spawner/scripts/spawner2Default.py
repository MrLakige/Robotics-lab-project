#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np

initialBlockPositions = [
		['X1-Y1-Z2', 0.418689, 0.541031, 0.863913],
		['X1-Y2-Z1', 0.373579, 0.735140, 0.863913], 
		['X1-Y2-Z2', 0.225137, 0.722706, 0.863913], 
		['X1-Y2-Z2-CHAMFER', 0.060266, 0.565019, 0.863913], 
		['X1-Y2-Z2-TWINFILLET', 0.079147, 0.730585, 0.863913],
		['X1-Y3-Z2', 0.152340, 0.701101, 0.863913], 
		['X1-Y3-Z2-FILLET', 0.245825, 0.426445, 0.863913], 
		['X1-Y4-Z1', 0.432769, 0.730745, 0.863913], 
		['X1-Y4-Z2', 0.279043, 0.649311, 0.863913], 
		['X2-Y2-Z2', 0.349389, 0.590763, 0.863913], 
		['X2-Y2-Z2-FILLET', 0.164513, 0.495294, 0.863913 ]
        ]

#delete a block by its name
def delete_model(name):
	delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	return delete_model_client(model_name=name)

#cleans the table in case there are blocks on it
for block in initialBlockPositions:	
		delete_model(f'{block[0]}')

for i in range(11):
    #Get a random lego block from all legos
    brick=initialBlockPositions[i][0]
    pos = Pose(Point(initialBlockPositions[i][1], initialBlockPositions[i][2], 0.870002), Quaternion(0, 0, 0, 0))
    #Call rospy spawn function to spawn objects in gazebo
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=''+str(brick), 
        model_xml=open('src/spawner/lego_models/'+brick+'/model.sdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=pos,
        reference_frame='world')



