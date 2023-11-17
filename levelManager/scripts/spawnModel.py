#!/bin/python3
'''
    Provides different plug and play visualizers from an urdf
    can be used as script to visualize the models from the command line
    author: G. Fadini
'''
import roslaunch
import tf
import numpy as np
import random
import sys
import xml.etree.ElementTree as ET
import rospy as ros
import rospkg
from rospy import Time

path = rospkg.RosPack().get_path("levelManager")

def spawnModel(model_name, pos, tf_broadcaster):
    package = 'gazebo_ros'
    executable = 'spawn_model'
    name = 'spawn_brick'
    namespace = '/'
    param_name = model_name+'_description'
    
    args = '-urdf -param '+param_name+' -model '+model_name+' -x '+  str(pos[0])+ ' -y '+  str(pos[1])+ ' -z '+  str(pos[2])
    node = roslaunch.core.Node(package, executable, name, namespace,args=args,output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    
    tf_broadcaster.sendTransform(pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(),'/'+model_name, '/world')

#color bricks
colorList = ['Gazebo/Indigo', 'Gazebo/Orange', \
		'Gazebo/Red', 'Gazebo/Purple', 'Gazebo/SkyBlue', \
		'Gazebo/DarkYellow', 'Gazebo/Green']

def changeModelColor(model_xml, color):
    root = ET.XML(model_xml)
    root.find('.//material/script/name').text = color
    return ET.tostring(root, encoding='unicode')

def changeMaterialColor(model_xml, color):
    print(f'{path}/urdf/{model_xml}.sdf')
    parser = xml.etree.XMLParser(recover=True)
    root= ET.fromstring(f'{path}/urdf/{model_xml}.sdf', parser=parser)

    # Find the <material> element within <visual>
    material_element = root.find('.//visual/material')

    # Update the name of the material
    if material_element is not None:
        name_element = material_element.find('name')
        if name_element is not None:
            name_element.text = color
        else:
            print("Name element not found within material.")
    else:
        print("Material element not found in the XML.")

    # Convert the modified XML tree back to a string
    modified_xacro_xml = ET.tostring(root, encoding='unicode')
    return modified_xacro_xml


if __name__ == '__main__':
    ros.init_node('spawn_node_python', anonymous=True)
    
    broadcaster1 = tf.TransformBroadcaster()    
    broadcaster2 = tf.TransformBroadcaster()    
    broadcaster3 = tf.TransformBroadcaster()    
    broadcaster4 = tf.TransformBroadcaster()    
    broadcaster5 = tf.TransformBroadcaster()    
    broadcaster6 = tf.TransformBroadcaster()    
    broadcaster7 = tf.TransformBroadcaster()    
    broadcaster8 = tf.TransformBroadcaster()    
    broadcaster9 = tf.TransformBroadcaster()    
    broadcaster10 = tf.TransformBroadcaster()    
    broadcaster11 = tf.TransformBroadcaster()    
    color = random.choice(colorList)
    changeMaterialColor('X1-Y1-Z2', color)
    spawnModel('X1-Y1-Z2', np.array([0.52, 0.25, 0.92]), broadcaster1)
    spawnModel('X1-Y2-Z1', np.array([0.70, 0.72, 0.92]), broadcaster2)
    spawnModel('X1-Y2-Z2', np.array([0.90, 0.64, 0.92]), broadcaster3)
    spawnModel('X1-Y2-Z2-CHAMFER', np.array([0.94, 0.27, 0.92]), broadcaster4)
    spawnModel('X1-Y2-Z2-TWINFILLET', np.array([0.90, 0.34, 0.92]), broadcaster5)
    spawnModel('X1-Y3-Z2', np.array([0.59, 0.27, 0.92]), broadcaster6)
    spawnModel('X1-Y3-Z2-FILLET', np.array([0.49, 0.27, 0.92]), broadcaster7)
    spawnModel('X1-Y4-Z1', np.array([0.63, 0.67, 0.92]), broadcaster8)
    spawnModel('X1-Y4-Z2', np.array([0.66, 0.63, 0.92]), broadcaster9)
    spawnModel('X2-Y2-Z2', np.array([0.78, 0.72, 0.92]), broadcaster10)
    spawnModel('X2-Y2-Z2-FILLET', np.array([0.83, 0.43, 0.92]), broadcaster11)
    
    ros.spin()
