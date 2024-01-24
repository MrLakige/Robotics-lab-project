#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import rospy
import os
import random

#delete a block by its name
def delete_model(name):
	delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	return delete_model_client(model_name=name)

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

#cleans the table in case there are blocks on it
for block in blocks:	
		delete_model(f'{block}')

#Generates random pose (posiiton and orientation)
pos = Pose(Point(random.uniform(0.060266, 0.432769), random.uniform(0.565019, 0.730745), 0.863913), Quaternion(0, 0, 0, 0))
#Get a random lego block from all legos
brick=blocks[random.randint(0,10)]
print(pos)
print(brick)
print(os. getcwd())
#Call rospy spawn function to spawn objects in gazebo
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name=''+str(brick), 
    model_xml=open('src/spawner/lego_models/'+brick+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')