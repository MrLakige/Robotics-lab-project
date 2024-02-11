"""
@package spawn
@brief spawn block in gazebo, level 3 with default values

@version 1.0
@author Hafsa, Michele, Sara
"""


#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np
from save import *

# Delete a Gazebo model by its name
def delete_model(name):
    """
    Delete a Gazebo model by its name.

    Args:
        name (str): The name of the model to be deleted.

    Returns:
        bool: True if the model deletion was successful, False otherwise.
    """
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    return delete_model_client(model_name=name)

# Array containing all Lego block names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

# Initial positions and orientations of Lego blocks
initialBlockPositions = [
		['X1-Y1-Z2', 0.280577, 0.691699, 0.870002, 0., 0., 0., 0.],
		['X1-Y2-Z1', 0.123776, 0.672390, 0.870002, 0., 0., 0., 0.],
		['X1-Y4-Z2', 0.414135, 0.725608, 0.870002, 0., 0., 0., 0.],
		['X1-Y3-Z2-FILLET', 0.367994, 0.598631, 0.900000, 0.5, -0.5, -0.5, 0.5]
		]

# Clean the table in case there are blocks on it
for block in blocks:
    delete_model(f'{block}')
    
clean()

# Spawn Lego blocks at predefined positions and orientations
for i in range(4):
    # Get a random Lego block from all Lego blocks
    brick = initialBlockPositions[i][0]
    pos = Pose(Point(initialBlockPositions[i][1], initialBlockPositions[i][2], initialBlockPositions[i][3]),
               Quaternion(initialBlockPositions[i][4], initialBlockPositions[i][5], initialBlockPositions[i][6], initialBlockPositions[i][7]))
    # Call rospy spawn function to spawn objects in Gazebo
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=''+str(brick), 
                       model_xml=open('../lego_models/'+brick+'/model.sdf', 'r').read(),
                       robot_namespace='/foo',
                       initial_pose=pos,
                       reference_frame='world')

    # Save information about the spawned Lego block to an XML file
    saveToXml(str(brick), pos, None)
