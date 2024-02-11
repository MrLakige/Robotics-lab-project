"""
@package spawn
@brief spawn block in gazebo, level 2 with default values

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

initialBlockPositions = [
	['X1-Y1-Z2', 0.924661, 0.541030, 0.863913],
	['X1-Y2-Z1', 0.233934, 0.734162, 0.863913], 
	['X1-Y2-Z2', 0.520516, 0.722642, 0.863913], 
	['X1-Y2-Z2-TWINFILLET', 0.179915, 0.577316, 0.863913],
	['X1-Y3-Z2', 0.329110, 0.700797, 0.863913], 
	['X1-Y3-Z2-FILLET', 0.431801, 0.723457, 0.863913], 
	['X1-Y4-Z1', 0.891367, 0.692272, 0.863913], 
	['X1-Y4-Z2', 0.619331, 0.656876, 0.863913], 
	['X2-Y2-Z2', 0.789534, 0.590818, 0.863913], 
	['X2-Y2-Z2-FILLET', 0.753272, 0.736383, 0.863913]
]

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

# Clean any residual data or files
clean()

# Clean the table by deleting any existing blocks
for block in initialBlockPositions:
    delete_model(f'{block[0]}')

# Generate random positions for Lego blocks
for i in range(6):
    # Get a specific Lego block and its initial position
    brick = initialBlockPositions[i][0]
    pos = Pose(Point(initialBlockPositions[i][1], initialBlockPositions[i][2], 0.870002), Quaternion(0, 0, 0, 0))
    
    # Call rospy spawn function to spawn Lego blocks in Gazebo
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
        model_name=''+str(brick),
        model_xml=open('../lego_models/'+brick+'/model.sdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=pos,
        reference_frame='world'
    )

    # Save information about the spawned Lego block to an XML file
    saveToXml(str(brick), pos, None)
