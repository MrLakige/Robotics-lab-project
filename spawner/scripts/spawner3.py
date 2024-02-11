"""
@package spawn
@brief spawn block in gazebo, level 3

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
positions = []

# List of possible rotations for Lego blocks
possibleRotations = [
    [0., 0., 0., 1], [0., 1., 0., 0.], 
    [0.5, 0.5, 0.5, 0.5], [-0.5, -0.5, 0.5, 0.5],
    [-0.5, 0.5, -0.5, 0.5], [0.5, -0.5, -0.5, 0.5],
    [0.7071, 0., 0., 0.7071], [0., 0.7071, -0.7071, 0.], 
    [1., 0., 0., 0.], [0., 0., 1., 0.],
    [-0.7071, 0., 0., 0.7071], [0., 0.7071, 0.7071, 0.]
]

# Clean the table in case there are blocks on it
for block in blocks:	
    delete_model(f'{block}')

clean()

# Generate random positions and rotations for Lego blocks
for i in range(4):
    f = True
    # Generate random position
    if i == 0:
        rotIndex = random.randint(0, 11)
        rot = possibleRotations[rotIndex]
        pos = Pose(Point(random.uniform(0.28, 0.44), random.uniform(0.565019, 0.730745), 0.870002), Quaternion(rot[0], rot[1], rot[2], rot[3]))
        if rotIndex in [2, 16]:  # Upside-down rotations
            pos.position.z = 0.92
        elif rotIndex in [5, 7, 11, 12, 14, 17, 20, 22]:
            pos.position.z = 0.90
        positions.append(pos)
    else:
        while f == True:
            pos = Pose(Point(random.uniform(0.28, 0.44), random.uniform(0.565019, 0.730745), 0.870002), Quaternion(0, 0, 0, 0))
            for k in range(i):
                threshold = 0.125
                if np.sqrt((pos.position.x-positions[k].position.x)**2 + (pos.position.y-positions[k].position.y)**2) < threshold:
                    break
                if k == i-1:
                    positions.append(pos)
                    f = False
    
    # Get a random Lego block from all Lego blocks
    brick = blocks[i]
    print(pos)
    print(brick)
    # Call rospy spawn function to spawn objects in Gazebo
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
