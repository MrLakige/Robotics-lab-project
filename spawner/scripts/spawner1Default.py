"""
@package spawn
@brief spawn block in gazebo, level 1 with default values

@version 1.0
@author Hafsa, Michele, Sara
"""


#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import rospy
import os
import random
from save import *

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

# The name of the Lego block to be deleted and then spawned again
block = 'X1-Y2-Z2'

# Delete the specified Lego block if it exists
delete_model(f'{block}')

# Clean any residual data or files
clean()

# Generate a predefined pose (position and orientation)
pos = Pose(Point(0.280577, 0.691699, 0.863913), Quaternion(0, 0, 0, 0))

# Set the Lego block name
brick = block
print(pos)
print(brick)
print(os.getcwd())

# Call rospy spawn function to spawn the Lego block in Gazebo
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
