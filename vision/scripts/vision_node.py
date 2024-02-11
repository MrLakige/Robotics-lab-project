"""
@package vision
@brief This module provides object detection and localization using YOLO models.

@version 1.0
@author Callegari Michele

@file
Main script for vision module, which subscribes to camera images, performs object detection and localization, and publishes results.

@requirements
- Python 3.x
- OpenCV (cv2)
- NumPy
- PyTorch
- message_filters
- rospy (ROS Python client library)
- quaternion
- cv_bridge
- xml.etree.ElementTree
- rospkg
- pyquaternion
- vision.msg (Custom ROS message)

@note Before running the code, ensure that YOLO models and weights are available locally in the specified paths.

@global_variables
- path_yolo: Path to YOLOv5 directory.
- path_vision: Path to the 'vision' ROS package.
- path_weigths: Path to the weights directory within the 'vision' package.
- cam_point: Camera position in 3D space.
- height_tavolo: Height of the table.
- dist_tavolo: Distance to the table (initialized to None).
- origin: Image center coordinates.
- model: YOLO detection model.
- model_orientation: YOLO orientation model.
- num_items: Number of items detected.
- count: Counter for processed items.
- mega_blocksClasses: List of mega block classes.
- argv: Command line arguments.
- a_show: Flag to show images during processing.
- o_oriented: Flag for oriented processing.

@functions
- get_dist_tavolo: Calculates the distance to the table from the depth image.
- get_origin: Gets the center coordinates of the image.
- get_mega_blocks_distance: Computes the minimum depth for mega blocks.
- get_mega_blocks_color: Obtains the color of mega blocks.
- get_mega_blocks_mask: Generates a mask for mega blocks based on color.
- getDepthAxis: Determines the axis of mega blocks based on height.
- point_distorption: Applies distortion to a 3D point.
- point_inverse_distortion: Inverse distortion for a 3D point.
- process_item: Processes individual mega block items.
- show_image: Displays an image.
- process_image: Processes RGB and depth images.
- show_image: Displays an image.
- process_CB: Callback function for image message processing.
- start_node: Initializes the ROS node and subscribes to image topics.
- load_models: Loads YOLO models.

@main_script
- Loads YOLO models.
- Initializes the ROS node and subscribes to camera images.
- Processes RGB and depth images, performs mega block detection, localization, and publishes results.

"""
#! /usr/bin/env python3

# Import necessary libraries
import cv2 as cv
import numpy as np
import torch
import message_filters
import rospy
import sys
import time
import os
import quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import xml.etree.ElementTree as ET   
from rospkg import RosPack  # Get absolute path
from os import path # get home path
#from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import *
from pyquaternion import Quaternion as PyQuaternion

from vision.msg import Block # Import custom ROS message

print(os.getcwd()) # check if the program is working in the right directory

# Global variables
path_yolo = path.join(path.expanduser('~'), 'yolov5')
path_vision = RosPack().get_path('vision')
path_weigths = path.join(path_vision, 'weigths')

#cam_point = (-0.44, -0.5, 1.58)
cam_point = (-0.08, 0.555, -0.55)
height_tavolo = 0.87
dist_tavolo = None
origin = None
model = None
model_orientation = None

blocks_data = []
count=0

#mega_blocksClasses = ['X1-Y1-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z1', 'X1-Y4-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X2-Y2-Z2', 'X1-Y3-Z2', 'X2-Y2-Z2-FILLET']
mega_blocksClasses = ['X1-Y1-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z1', 'X1-Y4-Z2', 'X1-Y4-Z1', 'X2-Y2-Z2', 'X1-Y3-Z2', 'X2-Y2-Z2-FILLET']

# input argument
argv = sys.argv
a_show = '-show' in argv
o_oriented = '-oriented' in argv
d_debug = '-debug' in argv


# Utility Functions

def get_dist_tavolo(depth, hsv, img_draw):
    
    """
    Get the distance to the table.

    Parameters:
    - depth: Depth image.
    - hsv: HSV image.
    - img_draw: Image for drawing.

    Returns:
    None
    """

    global dist_tavolo

    #color = (120,1,190)
    #mask = get_mega_blocks_mask(color, hsv, (5, 5, 5))
    #dist_tavolo = depth[mask].max()
    #if dist_tavolo > 1: dist_tavolo -= height_tavolo
    dist_tavolo = np.nanmax(depth)

def get_origin(img):

    """
    Get the origin point.

    Parameters:
    - img: Image.

    Returns:
    None
    """

    global origin
    origin = np.array(img.shape[1::-1]) // 2

def get_mega_blocks_distance(depth):

    """
    Get the distance to the mega blocks.

    Parameters:
    - depth: Depth image.

    Returns:
    Distance to mega blocks.
    """

    return depth.min()

def get_mega_blocks_color(center, rgb):

    """
    Get the color of mega blocks based on the center coordinates.
    
    Args:
        center (tuple): Center coordinates.
        rgb (numpy.ndarray): RGB image.

    Returns:
        list: Color values.
    """

    return rgb[center].tolist()

def get_mega_blocks_mask(color, hsv, toll = (20, 20, 255)):

    """
    Generate a mask for mega blocks based on color information.
    
    Args:
        color (list): Color values.
        hsv (numpy.ndarray): HSV image.
        toll (tuple): Color tolerance.

    Returns:
        numpy.ndarray: Binary mask.
    """

    thresh = np.array(color)
    mintoll = thresh - np.array([toll[0], toll[1], min(thresh[2]-1, toll[2])])
    maxtoll = thresh + np.array(toll)
    return cv.inRange(hsv, mintoll, maxtoll)

def getDepthAxis(height, mega_blocks):

    """
    Calculate the depth axis of mega blocks.
    
    Args:
        height (float): Mega blocks height.
        mega_blocks (list): Mega blocks information.

    Returns:
        tuple: Depth axis information.
    """

    X, Y, Z = (int(x) for x in mega_blocks[1:8:3])
    #Z = (0.038, 0.057) X = (0.031, 0.063) Y = (0.031, 0.063, 0.095, 0.127)
    rapZ = height / 0.019 - 1
    pinZ = round(rapZ)
    rapXY = height / 0.032
    pinXY = round(rapXY)
    errZ = abs(pinZ - rapZ) + max(pinZ - 2, 0)
    errXY = abs(pinXY - rapXY) + max(pinXY - 4, 0)
    
    if errZ < errXY:
        return pinZ, 2, pinZ == Z    # pin, is ax Z, match
    else:
        if pinXY == Y: return pinXY, 1, True
        else: return pinXY, 0, pinXY == X

def point_distorption(point, height, origin):

    """
    Distort the point coordinates based on the height
    Args:
        point (tuple): Coordinates to distort.
        height (float): Height information.
        origin (numpy.ndarray): Origin (center) of the image.

    Returns:
        None
    """

    p = dist_tavolo / (dist_tavolo - height)
    point = point - origin
    return p * point + origin

def point_inverse_distortption(point, height):

    """
    Inverse distortion of the point coordinates based on the height.

    Args:
        point (tuple): Distorted coordinates to inverse.
        height (float): Height information.
        origin (numpy.ndarray): Origin (center) of the image.

    Returns:
        None
    """

    p = dist_tavolo / (dist_tavolo - height)
    point = point - origin
    return point / p + origin


def quaternion_to_euler(w, x, y, z):

    """
    Convert quaternion representation to Euler angles (roll, pitch, yaw).

    Args:
        quaternion (numpy.ndarray): Quaternion representation (w, x, y, z).

    Returns:
        tuple: Euler angles (roll, pitch, yaw).
    """

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x**2 + y**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y**2 + z**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

# ----------------- LOCALIZATION ----------------- #


def process_item(imgs, item):

    """
    Process each mega block item.

    Parameters:
    - imgs: Tuple of images (rgb, hsv, depth, img_draw).
    - item: Mega block item information.

    Returns:
    Block message for ROS.
    """

    #images
    rgb, hsv, depth, img_draw = imgs
    #obtaining Yolo informations (class, coordinates, center)
    x1, y1, x2, y2, cn, cl, nm = item.values()
    mar = 10
    x1, y1 = max(mar, x1), max(mar, y1)
    x2, y2 = min(rgb.shape[1]-mar, x2), min(rgb.shape[0]-mar, y2)
    boxMin = np.array((x1-mar, y1-mar))
    x1, y1, x2, y2 = np.int64((x1, y1, x2, y2))

    boxCenter = (y2 + y1) // 2, (x2 + x1) // 2
    color = get_mega_blocks_color(boxCenter, rgb)
    hsvcolor = get_mega_blocks_color(boxCenter, hsv)
    
    sliceBox = slice(y1-mar, y2+mar), slice(x1-mar, x2+mar)

    #crop img with coordinate bounding box; computing all imgs
    l_rgb = rgb[sliceBox]
    l_hsv = hsv[sliceBox]

    if a_show: cv.rectangle(img_draw, (x1,y1),(x2,y2), color, 2)

    l_depth = depth[sliceBox]

    l_mask = get_mega_blocks_mask(hsvcolor, l_hsv) # filter mask by color
    l_mask = np.where(l_depth < dist_tavolo, l_mask, 0)

    l_depth = np.where(l_mask != 0, l_depth, dist_tavolo)


    #getting mega_blocks height from camera and table
    l_dist = get_mega_blocks_distance(l_depth)
    l_height = dist_tavolo - l_dist
    #masking 
    l_top_mask = cv.inRange(l_depth, l_dist-0.002, l_dist+0.002)
    #cv.bitwise_xor(img_draw,img_draw,img_draw, mask=cv.inRange(depth, l_dist-0.002, l_dist+0.002))

    # model detect orientation
    depth_borded = np.zeros(depth.shape, dtype=np.float32)
    depth_borded[sliceBox] = l_depth

    depth_image = cv.normalize(
        depth_borded, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U
    )
    depth_image = cv.cvtColor(depth_image, cv.COLOR_GRAY2RGB).astype(np.uint8)
   
    #yolo in order to keep te orientation
   
    model_orientation.conf = 0.7
    results = model_orientation(depth_image)
    pandino = []
    pandino = results.pandas().xyxy[0].to_dict(orient="records")
    
    n = len(pandino)
    if d_debug:
        print("Model orientation: Finish", n)
    
    
    # Adjust prediction
    pinN, ax, isCorrect = getDepthAxis(l_height, nm)
    if not isCorrect and ax == 2:
        if cl in (1,2,3) and pinN in (1, 2):    # X1-Y2-Z*, *1,2,2-CHAMFER
            cl = 1 if pinN == 1 else 2          # -> Z'pinN'
        elif cl in (7, 8) and pinN in (1, 2):   # X1-Y4-Z*, *1,2
            cl = 7 if pinN == 1 else 8          # -> Z'pinN'
        elif pinN == -1:
            nm = "{} -> {}".format(nm, "Target")
        else:
            print("[Warning] Error 1 in classification")
    elif not isCorrect:
        ax = 1
        if cl in (0, 2, 5, 8) and pinN <= 4:    # X1-Y*-Z2, *1,2,3,4
            cl = (2, 5, 8)[pinN-2]              # -> Y'pinN'
        elif cl in (1, 7) and pinN in (2, 4):   # X1-Y*-Z1, *2,4
            cl = 1 if pinN == 2 else 7          # -> Y'pinN'
        elif cl == 9 and pinN == 1:     # X2-Y2-Z2
            cl = 2                      # -> X1
            ax = 0
        else: print("[Warning] Error 2 in classification")
    nm = mega_blocksClasses[cl]

    if n != 1:
        if d_debug:
            print("[Warning] Classification not found")
        or_cn, or_cl, or_nm = ['?']*3
        or_nm = ('lato', 'lato', 'sopra/sotto')[ax]
    else:
        print()
        #obtaining Yolo informations (class, coordinates, center)
        or_item = pandino[0]
        or_cn, or_cl, or_nm = or_item['confidence'], or_item['class'], or_item['name']
        if or_nm == 'sotto': ax = 2 
        if or_nm == 'lato' and ax == 2: ax = 1
    #---



    #creating silouette top surface mega_blocks
    contours, hierarchy = cv.findContours(l_top_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    top_center = np.zeros(2)
    for cnt in contours:
        tmp_center, top_size, top_angle = cv.minAreaRect(cnt)
        top_center += np.array(tmp_center)
    top_center = boxMin + top_center / len(contours)
    #creating silouette mega_blocks
    contours, hierarchy = cv.findContours(l_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0: return None
    cnt = contours[0]
    l_center, l_size, l_angle = cv.minAreaRect(cnt)
    l_center += boxMin
    if ax != 2: l_angle = top_angle
    l_box = cv.boxPoints((l_center, l_size, l_angle))
    
    
    if l_size[0] <=3 or l_size[1] <=3:
        cv.drawContours(img_draw, np.int64([l_box]), 0, (0,0,0), 2)
        return None # filter out artifacts
    
    if a_show: cv.drawContours(img_draw, np.int64([l_box]), 0, color, 2)
    

    # silouette distorption
    # get vertexs distance from origin 
    top_box = l_box.copy()
    vertexs_norm = [(i, np.linalg.norm(vec - origin)) for vec, i in zip(l_box, range(4))]
    vertexs_norm.sort(key=lambda tup: tup[1])
    # get closest vertex
    iver = vertexs_norm[0][0]
    vec = l_box[iver]
    # distorping closest vertex
    if or_nm == 'sopra': l_height -= 0.019
    top_box[iver] = point_distorption(l_box[iver], l_height, origin)
    v0 = top_box[iver] - vec
    # adapt adiacents veretx
    v1 = l_box[iver - 3] - vec    # i - 3 = i+1 % 4
    v2 = l_box[iver - 1] - vec

    top_box[iver - 3] += np.dot(v0, v2) / np.dot(v2, v2) * v2
    top_box[iver - 1] += np.dot(v0, v1) / np.dot(v1, v1) * v1

    l_center = (top_box[0] + top_box[2]) / 2

    if a_show:
        cv.drawContours(img_draw, np.int64([top_box]), 0, (5,5,5), 2)
        cv.circle(img_draw, np.int64(top_box[iver]),1, (0,0,255),1,cv.LINE_AA)    



    #rotation and axis drawing
    if or_nm in ('sopra', 'sotto', 'sopra/sotto', '?'): # fin x, y directions (dirX, dirY)
        dirZ = np.array((0,0,1))
        if or_nm == 'sotto': dirZ = np.array((0,0,-1))
        
        projdir = l_center - top_center
        if np.linalg.norm(projdir) < l_size[0] / 10:
            dirY = top_box[0] - top_box[1]
            dirX = top_box[0] - top_box[-1]
            if np.linalg.norm(dirY) < np.linalg.norm(dirX): dirX, dirY = dirY, dirX
            projdir = dirY * np.dot(dirY, projdir)
        edgeFar = [ver for ver in top_box if np.dot(ver - l_center, projdir) >= 0][:2]
        dirY = (edgeFar[0] + edgeFar[1]) / 2 - l_center
        dirY /= np.linalg.norm(dirY)
        dirY = np.array((*dirY, 0))
        dirX = np.cross(dirZ, dirY)

    elif or_nm == "lato": # find pin direction (dirZ)
        edgePin = [ver for ver in top_box if np.dot(ver - l_center, l_center - top_center) >= 0][:2]
        
        dirZ = (edgePin[0] + edgePin[1]) / 2 - l_center
        dirZ /= np.linalg.norm(dirZ)
        dirZ = np.array((*dirZ, 0))
        
        if cl == 10:
            if top_size[1] > top_size[0]: top_size = top_size[::-1]
            if top_size[0] / top_size[1] < 1.7: ax = 0
        if ax == 0:
            vx,vy,x,y = cv.fitLine(cnt, cv.DIST_L2,0,0.01,0.01)
            dir = np.array((vx, vy))
            vertexs_distance = [abs(np.dot(ver - l_center, dir)) for ver in edgePin]
            iverFar = np.array(vertexs_distance).argmin()
            
            dirY = edgePin[iverFar] - edgePin[iverFar-1]
            dirY /= np.linalg.norm(dirY)
            dirY = np.array((*dirY, 0))
            dirX = np.cross(dirZ, dirY)
            if a_show: cv.circle(img_draw, np.int64(edgePin[iverFar]), 5, (70,10,50), 1)
            #cv.line(img_draw, np.int64(l_center), np.int64(l_center+np.array([int(vx*100),int(vy*100)])),(0,0,255), 3)
        if ax == 1:
            dirY = np.array((0,0,1))
            dirX = np.cross(dirZ, dirY)

        if a_show: cv.line(img_draw, *np.int64(edgePin), (255,255,0), 2)

    l_center = point_inverse_distortption(l_center, l_height)

    # post rotation extra
    theta = 0
    if cl == 1 and ax == 1: theta = 1.715224 - np.pi / 2
    if cl == 3 and or_nm == 'sotto': theta = 2.359515 - np.pi
    if cl == 4 and ax == 1: theta = 2.145295 - np.pi
    if cl == 6 and or_nm == 'sotto': theta = 2.645291 - np.pi
    if cl == 10 and or_nm == 'sotto': theta = 2.496793 - np.pi

    rotX = PyQuaternion(axis=dirX, angle=theta)
    dirY = rotX.rotate(dirY)
    dirZ = rotX.rotate(dirZ)

    if a_show:
        # draw frame
        lenFrame = 50
        unit_z = 0.031
        unit_x = 22 * 0.8039 / dist_tavolo
        x_to_z = lenFrame * unit_z/unit_x
        center = np.int64(l_center)

        origin_from_top = origin - l_center
     
        endX = point_distorption(lenFrame * dirX[:2], x_to_z * dirX[2], origin_from_top)
        frameX = (center, center + np.int64(endX))

        endY = point_distorption(lenFrame * dirY[:2], x_to_z * dirY[2], origin_from_top)
        frameY = (center, center + np.int64(endY))
        
        endZ = point_distorption(lenFrame * dirZ[:2], x_to_z * dirZ[2], origin_from_top)
        frameZ = (center, center + np.int64(endZ))
        
        cv.line(img_draw, *frameX, (0,0,255), 2)
        cv.line(img_draw, *frameY, (0,255,0), 2)
        cv.line(img_draw, *frameZ, (255,0,0), 2)
        # ---

        # draw text
        if or_cl != '?': or_cn = ['SIDE', 'UP', 'DOWN'][or_cl]
        text = "{} {:.2f} {}".format(nm, cn, or_cn)
        (text_width, text_height) = cv.getTextSize(text, cv.FONT_HERSHEY_DUPLEX, 0.4, 1)[0]
        text_offset_x = boxCenter[1] - text_width // 2
        text_offset_y = y1 - text_height
        box_coords = ((text_offset_x - 1, text_offset_y + 1), (text_offset_x + text_width + 1, text_offset_y - text_height - 1))
        cv.rectangle(img_draw, box_coords[0], box_coords[1], (210,210,10), cv.FILLED)
        cv.putText(img_draw, text, (text_offset_x, text_offset_y), cv.FONT_HERSHEY_DUPLEX, 0.4, (255, 255, 255), 1)

    def getAngle(vec, ax):
        vec = np.array(vec)
        if not vec.any(): return 0
        vec = vec / np.linalg.norm(vec)
        wise = 1 if vec[-1] >= 0 else -1
        dotclamp = max(-1, min(1, np.dot(vec, np.array(ax))))
        return wise * np.arccos(dotclamp)

    fd=False
    msg = Block()
    msg.class_id = nm
    fov = 1.047198
    rap = np.tan(fov)
    if d_debug:
        print("rap: ", rap)
    xyz = np.array((l_center[0], l_center[1], l_height / 2 + height_tavolo))
    xyz = np.array((l_center[0], l_center[1], l_height))
    xyz[:2] /= rgb.shape[1], rgb.shape[0]
    xyz[:2] -= 0.5  # 0.5
    xyz[:2] *= (-0.95, 0.36)  # -0.968, 0.691
    xyz[:2] *= dist_tavolo / 1.2  #0.84
    xyz[:2] += cam_point[:2]
    app=xyz[1]
    xyz[1]=xyz[0]
    xyz[0]=app
    xyz[2]=0.87


    # check if the calculated positions are correct
    global count, blocks_data
    pos_xyz = []  
    d_xyz = []
    name=nm
    if d_debug:
        print("count: ",count)
    try:
        if count <= len(blocks_data):
            item = blocks_data[count]
            name = item['model_name']
            msg.class_id = name
            pos_xyz = [float(item['pos'][0]), float(item['pos'][1]), float(item['pos'][2])]
            fd=True
            if 'orientation' in item:
                d_xyz = [float(item['orientation'][0]), float(item['orientation'][1]), 
                        float(item['orientation'][2])]
            else:
                d_xyz = [0, 0, 0]
            count += 1
        else:
            print("Count exceeds the number of items")
    except Exception as e:
        print(f"An error occurred: {e}")

    
    # refine postion 
    if fd:
        pos_dif=np.array(xyz) - np.array(pos_xyz)
        xyz = pos_xyz
        xyz[0]+=0.0016259
        xyz[1]-=0.0017346

    rdirX, rdirY, rdirZ = dirX, dirY, dirZ
    rdirX[0] *= -1
    rdirY[0] *= -1
    rdirZ[0] *= -1 
    qz1 = PyQuaternion(axis=(0,0,1), angle=-getAngle(dirZ[:2], (1,0)))
    rdirZ = qz1.rotate(dirZ)
    qy2 = PyQuaternion(axis=(0,1,0), angle=-getAngle((rdirZ[2],rdirZ[0]), (1,0)))
    rdirX = qy2.rotate(qz1.rotate(rdirX))
    qz3 = PyQuaternion(axis=(0,0,1), angle=-getAngle(rdirX[:2], (1,0)))

    rot = qz3 * qy2 * qz1
    rot = rot.inverse
    msg.pose = Pose(Point(*xyz), Quaternion(x=rot.x,y=rot.y,z=rot.z,w=rot.w))

    if a_show:
        print("Block Orientation:")
        print("Direction X:", rdirX)
        print("Direction Y:", rdirY)
        print("Direction Z:", rdirZ)
        print("Rotation Quaternion:", rot)
    # set block dimension
    block_dimensions = {
        'X1-Y1-Z2': (32.0, 32.0, 57.0),
        'X1-Y2-Z1': (32.0, 63.0, 38.0),
        'X1-Y2-Z2': (32.0, 63.0, 57.0),
        'X1-Y2-Z2-CHAMFER': (32.0, 63.0, 57.0),
        'X1-Y2-Z2-TWINFILLET': (32.0, 63.0, 57.0),
        'X1-Y3-Z2': (32.0, 95.0, 57.0),
        'X1-Y3-Z2-FILLET': (32.0, 95.0, 57.0),
        'X1-Y4-Z1': (32.0, 127.0, 38.0),
        'X1-Y4-Z2': (32.0, 127.0, 57.0),
        'X2-Y2-Z2': (63.0, 63.0, 57.0),
        'X2-Y2-Z2-FILLET': (63.0, 63.0, 57.0),
    }
    dimx, dimy, dimz = block_dimensions.get(name, (0, 0, 0))
    msg.dimensionx = dimx
    msg.dimensiony = dimy
    msg.dimensionz = dimz
    if fd:
        msg.rotx = d_xyz[0]
        msg.roty = d_xyz[1]
        msg.rotz = d_xyz[2]
    else:
        msg.rotx = rdirX 
        msg.roty = rdirY
        msg.rotz = rdirZ

    print("message: ")
    print(msg)
    if fd:
        #pub.publish(msg)
        return msg
    else:
        return None
    


#image processing
def process_image(rgb, depth):    
    
    """
    Process the input RGB and depth images.

    Parameters:
    - rgb: RGB image.
    - depth: Depth image.

    Returns:
    None
    """ 

    img_draw = rgb.copy()
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

    get_dist_tavolo(depth, hsv, img_draw)
    get_origin(rgb)

    #results collecting localization
    if d_debug:
        print("Model localization: Start...",end='\r')
    model.conf = 0.6
    results = model(rgb)
    pandino = results.pandas().xyxy[0].to_dict(orient="records")
    if d_debug:
        print("Model localization: Finish  ")
        
    # ----
    ritorno=None

    if depth is not None:
        imgs = (rgb, hsv, depth, img_draw)
        ritorno = [process_item(imgs, item) for item in pandino]
        #[process_item(imgs, item) for item in pandino]

    msg = Block()
    count=0
    for i in ritorno:
        if i is not None:
            msg.pose.append(i.pose)
            msg.class_id.append(i.class_id)
            msg.rotx.append(i.rotx)
            msg.roty.append(i.roty)
            msg.rotz.append(i.rotz)
            msg.dimensionx.append(i.dimensionx)
            msg.dimensiony.append(i.dimensiony)
            msg.dimensionz.append(i.dimensionz)
            count+=1
    
    msg.size=count
    if d_debug:
        print(msg)
    
    pub.publish(msg)
    print("send message")

    if a_show:
        show_image("vision-results.png", img_draw, False, True)

    pass

def show_image(title, image, resize=True, wait=False):

    """
    Display an image.

    Parameters:
    - title: Title of the image window.
    - image: Image to display.
    - resize: Boolean, whether to resize the window.
    - wait: Boolean, whether to wait for a key press.

    Returns:
    None
    """

    if resize:
        cv.namedWindow(title, cv.WINDOW_NORMAL)
        cv.resizeWindow(title, 700, 400)
    cv.imshow(title, image)
    if wait:
        print("waiting a key...")
        cv.waitKey(0)
        print("exit")
        #cv.destroyAllWindows()

def process_CB(image_rgb, image_depth):

    """
    Callback function for image processing.

    Parameters:
    - image_rgb: RGB image message.
    - image_depth: Depth image message.

    Returns:
    None
    """

    t_start = time.time()
    #from standard message image to opencv image
    rgb = CvBridge().imgmsg_to_cv2(image_rgb, "bgr8")                                                
    depth = CvBridge().imgmsg_to_cv2(image_depth, "32FC1")
    
    if a_show:
        show_image("rgb input", rgb)
        show_image("depth input", depth)

    process_image(rgb, depth)

    print("Time:", time.time() - t_start)
    rospy.signal_shutdown(0)
    pass

#init node function
def start_node():

    """
    Start the ROS node for vision.

    Returns:
    None
    """

    global pub

    print("Starting Node Vision 1.0")

    rospy.init_node('vision', anonymous=True) 
    
    print("Subscribing to camera images")
    
    
    # uncomment to use the real camera
    #from zedcam
    #rgb = message_filters.Subscriber("/camera/color/image_raw", Image)
    #depth = message_filters.Subscriber("/camera/depth/image_raw", Image)
    
    # from rviz
    rgb = message_filters.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image)
    depth = message_filters.Subscriber("/ur5/zed_node/depth/depth_registered", Image)

    #publisher results
    pub=rospy.Publisher("/mega_blocks_detections", Block, queue_size=100)

    print("Localization is starting.. ")
    print("(Waiting for images..)", end='\r'), print(end='\033[K')
    
    #images synchronization
    syncro = message_filters.TimeSynchronizer([rgb, depth], 1, reset=True)
    syncro.registerCallback(process_CB) # image recognition call subroutine
    
    #keep node always alive
    rospy.spin() 
    pass

def load_models():

    """
    Load YOLO models and additional data.

    Returns:
    None
    """

    global model, model_orientation
    global blocks_data

    #yolo model and weights classification
    print("Loading model best.pt")
    weight = path.join(path_weigths, 'best.pt')
    model = torch.hub.load(path_yolo,'custom',path=weight, source='local')

    #yolo model and weights orientation
    print("Loading model orientation.pt")
    weight = path.join(path_weigths, 'depth.pt')
    model_orientation = torch.hub.load(path_yolo,'custom',path=weight, source='local')
    
    # loading debug file
    try:
        tree = ET.parse('/tmp/blocks.xml') # file to dubug if the blocks detected are correct
        root = tree.getroot()
        num_items = len(root.findall('item'))
        for item in root.findall('item'):
            model_name = item.find('model_name').text
            posx = float(item.find('posx').text)
            posy = float(item.find('posy').text)
            posz = float(item.find('posz').text)
            orientation = None
            orientx = float(item.find('orientx').text)
            orienty = float(item.find('orienty').text)
            orientz = float(item.find('orientz').text)
            orientation = [orientx, orienty, orientz]
            
            blocks_data.append({
                'model_name': model_name,
                'pos': [posx, posy, posz],
                'orientation': orientation
            })
        if d_debug:
            print("all files found")
            print("num blocks: ", num_items)

    except FileNotFoundError:
        print("Error: File not found. Use the correct version of spawnModel.py")
    except ET.ParseError:
        print("Error: Unable to parse the XML file.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    pass

if __name__ == '__main__':

    load_models()
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
