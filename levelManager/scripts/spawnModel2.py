#!/bin/python3
'''
    Provides different plug and play visualizers from an urdf
    can be used as script to visualize the models from the command line
    author: G. Fadini
'''
import random
import roslaunch
import tf
import xml.etree.ElementTree as ET
import numpy as np
import rospy as ros
from rospy import Time
import sys
from enum import Enum



class ModelName(Enum):
    X1_Y1_Z2 = 'X1-Y1-Z2'
    X1_Y2_Z1 = 'X1-Y2-Z1'
    X1_Y2_Z2 = 'X1-Y2-Z2'
    X1_Y2_Z2_CHAMFER = 'X1-Y2-Z2-CHAMFER'
    X1_Y2_Z2_TWINFILLET = 'X1-Y2-Z2-TWINFILLET'
    X1_Y3_Z2 = 'X1-Y3-Z2'
    X1_Y3_Z2_FILLET = 'X1-Y3-Z2-FILLET'
    X1_Y4_Z1 = 'X1-Y4-Z1'
    X1_Y4_Z2 = 'X1-Y4-Z2'
    X2_Y2_Z2 = 'X2-Y2-Z2'
    X2_Y2_Z2_FILLET = 'X2-Y2-Z2-FILLET'

def spawnModel(model_name, tf_broadcaster, pos=None, orientation=(0.0, 0.0, 0.0, 1.0)):
    if pos is None:
        # Generate a random numpy array between 0.4 and 0.7 for x and y, and keep z constant
        pos = np.append(np.random.uniform(0.4, 0.7, size=(2,)), 0.92)
        pos = np.append(pos, 0.92)  # Set the z-coordinate to the specified value
    
    package = 'gazebo_ros'
    executable = 'spawn_model'
    name = 'spawn_brick'
    namespace = '/'
    param_name = model_name.value + '_description'
    
    args = '-urdf -param ' + param_name + ' -model ' + model_name.value + ' -x ' + str(pos[0]) + ' -y ' + str(pos[1]) + ' -z ' + str(pos[2])
    node = roslaunch.core.Node(package, executable, name, namespace,args=args,output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    
    tf_broadcaster.sendTransform(pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/' + model_name.value, '/world')


def saveToXml(model_name, pos, orientation):
    root = ET.Element("item")
    
    model_name_element = ET.SubElement(root, "model_name")
    model_name_element.text = "block:" + model_name.value
    
    posx_element = ET.SubElement(root, "posx")
    posx_element.text = str(pos[0])
    
    posy_element = ET.SubElement(root, "posy")
    posy_element.text = str(pos[1])
    
    posz_element = ET.SubElement(root, "posz")
    posz_element.text = str(pos[2])
    
    if orientation is not None:
        orient_x_element = ET.SubElement(root, "orientx")
        orient_x_element.text = str(orientation[0])
        
        orient_y_element = ET.SubElement(root, "orienty")
        orient_y_element.text = str(orientation[1])
        
        orient_z_element = ET.SubElement(root, "orientz")
        orient_z_element.text = str(orientation[2])
        
        orient_w_element = ET.SubElement(root, "orientw")
        orient_w_element.text = str(orientation[3])
        
    tree = ET.ElementTree(root)
    tree.write("/tmp/blocks.xml")


def spawnRandomBlocksLevel1():
    broadcaster = tf.TransformBroadcaster()
    pos = np.array([np.random.uniform(0.45, 0.7), np.random.uniform(0.45, 0.7), 0.92])

    # 1st level: Spawn 1 block with random position
    random_model = random.choice(list(ModelName))
    spawnModel(random_model, broadcaster, pos)
    
    # Save data to XML file
    saveToXml(random_model, pos, None)

def spawnRandomBlocksLevel2(num_blocks):
    broadcaster = tf.TransformBroadcaster()
    used_positions = set()

    # 2nd level: Spawn random number of blocks with random positions
    for _ in range(num_blocks):
        random_model = random.choice(list(ModelName))

        while True:
            pos = np.array([np.random.uniform(0.45, 0.7), np.random.uniform(0.45, 0.7), 0.92])

            if tuple(pos) not in used_positions:
                used_positions.add(tuple(pos))
                break

        spawnModel(random_model, broadcaster, pos)
        saveToXml(random_model, pos, None)

def spawnRandomBlocksLevel3(num_blocks):
    broadcaster = tf.TransformBroadcaster()
    used_positions = set()

    # 3rd level: Spawn random number of blocks with random positions and random orientation
    for _ in range(num_blocks):
        random_model = random.choice(list(ModelName))

        while True:
            pos = np.array([np.random.uniform(0.45, 0.7), np.random.uniform(0.45, 0.7), 0.92])

            if tuple(pos) not in used_positions:
                used_positions.add(tuple(pos))
                break

        orientation = np.random.rand(3)
        orientation = np.append(orientation, 1.0)  # Quaternion representation (x, y, z, w)
        spawnModel(random_model, broadcaster, pos, orientation)
        saveToXml(random_model, pos, orientation)

if __name__ == '__main__':
    ros.init_node('spawn_node_python', anonymous=True)

    # Check for command-line arguments
    if len(sys.argv) != 2:
        print("Usage: python script.py <level>")
        sys.exit(1)

    level = int(sys.argv[1])
    num_blocks = random.randint(1, 10)

    if level == 1:
        spawnRandomBlocksLevel1()
    elif level == 2:
        spawnRandomBlocksLevel2(num_blocks)
    elif level == 3:
        spawnRandomBlocksLevel3(num_blocks)
    else:
        print("Invalid level. Please choose 1, 2, or 3.")
        sys.exit(1)

    ros.spin()