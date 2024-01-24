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
block = 'X1-Y2-Z2'

#cleans the table in case there are blocks on it
delete_model(f'{block}')

#Generates random pose (posiiton and orientation)
pos = Pose(Point(0.280577, 0.691699, 0.863913), Quaternion(0, 0, 0, 0))
#Get a random lego block from all legos
brick=block
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
