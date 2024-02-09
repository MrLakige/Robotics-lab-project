import tf
import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
import sys
import os
import math

def quaternionToEulerAngles(quaternion):
    r = Rotation.from_quat(quaternion)
    euler_angles = r.as_euler('xyz', degrees=True)
    return euler_angles

def clean():
    existing_file_path = "/tmp/blocks.xml"
    if os.path.exists(existing_file_path):
        os.remove(existing_file_path)

def saveToXml(model_name, pos, orientation):
    try:
        tree = ET.parse("/tmp/blocks.xml")
        root = tree.getroot()
    except FileNotFoundError:
        root = ET.Element("root")
        tree = ET.ElementTree(root)

    item = ET.SubElement(root, "item")

    model_name_element = ET.SubElement(item, "model_name")
    model_name_element.text = model_name

    posx_element = ET.SubElement(item, "posx")
    posx_element.text = str(pos.position.x)

    posy_element = ET.SubElement(item, "posy")
    posy_element.text = str(pos.position.y)

    posz_element = ET.SubElement(item, "posz")
    posz_element.text = str(pos.position.z)

    if orientation is not None:
        euler_angles = quaternionToEulerAngles(orientation)
        orient_x_element = ET.SubElement(item, "orientx")
        orient_x_element.text = str(euler_angles[0])

        orient_y_element = ET.SubElement(item, "orienty")
        orient_y_element.text = str(euler_angles[1])

        orient_z_element = ET.SubElement(item, "orientz")
        orient_z_element.text = str(euler_angles[2])

    else:
        orient_x_element = ET.SubElement(item, "orientx")
        orient_x_element.text = str(0)

        orient_y_element = ET.SubElement(item, "orienty")
        orient_y_element.text = str(0)

        orient_z_element = ET.SubElement(item, "orientz")
        orient_z_element.text = str(0)

    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
    with open("/tmp/blocks.xml", "w") as xml_file:
        xml_file.write(xml_str)