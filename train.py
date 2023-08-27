from imageai.Detection.Custom import DetectionModelTrainer

trainer = DetectionModelTrainer()
trainer.setModelTypeAsYOLOv3()
trainer.setDataDirectory(data_directory="blocks")
trainer.setTrainConfig(object_names_array=["blocks"], batch_size=5, num_experiments=100, train_from_pretrained_model="yolov3.pt")
trainer.trainModel()