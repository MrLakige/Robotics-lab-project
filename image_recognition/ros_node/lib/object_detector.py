import os
import sys
from imageai.Detection import ObjectDetection
import json

# Check if the correct number of arguments is provided
if len(sys.argv) != 2:
    print("Usage: python script_name.py input_image_path")
    sys.exit(1)

# Get the input image path from the command-line argument
input_image_path = sys.argv[1]

execution_path = os.getcwd()

detector = ObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath(os.path.join(execution_path, "yolov3.pt"))
detector.loadModel()
detections = detector.detectObjectsFromImage(input_image=input_image_path, output_image_path=os.path.join(execution_path, "imageNew.jpg"), minimum_percentage_probability=30)

message = []
for eachObject in detections:
    box_points = eachObject["box_points"]
    x1, y1, x2, y2 = box_points
    message.append({
        'name': eachObject["name"],
        'probability': eachObject["percentage_probability"],
        'x1': x1,
        'y1': y1,
        'x2': x2,
        'y2': y2
    })

# Serialize the JSON data to a string
json_data = json.dumps(message)

# Save the JSON message to a file
output_file_name = "objects.json"
with open(output_file_name, "w") as output_file:
    output_file.write(json_data)
    print(f"Message saved to {output_file_name}")

exit(0)
