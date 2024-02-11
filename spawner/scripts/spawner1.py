"""
@package spawn
@brief spawn block in gazebo, level 1

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

# Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

# Cleans the table by deleting any existing blocks
for block in blocks:	
    delete_model(f'{block}')

clean()

# Generates random pose (position and orientation)
pos = Pose(Point(random.uniform(0.28, 0.44), random.uniform(0.565019, 0.730745), 0.863913), Quaternion(0, 0, 0, 0))

# Get a random lego block from all legos
brick = blocks[random.randint(0, 10)]
print(pos)
print(brick)
print(os.getcwd())

# Call rospy spawn function to spawn objects in Gazebo
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(
    model_name=''+str(brick),
    model_xml=open('../lego_models/'+brick+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world'
)

saveToXml(str(brick), pos, None)
