from imageai.Detection.Custom import CustomObjectDetection

detector = CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath("yolov3_blocks_mAP-0.00894_epoch-6.pt")
detector.setJsonPath("blocks_yolov3_detection_config.json")
detector.loadModel()
detections = detector.detectObjectsFromImage(input_image="test2.png", output_image_path="output_test.jpg")
for detection in detections:
    print(detection["name"], " : ", detection["percentage_probability"], " : ", detection["box_points"])