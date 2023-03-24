from imageai.Classification.Custom import ClassificationModelTrainer

model_trainer = ClassificationModelTrainer()
model_trainer.setModelTypeAsInceptionV3()
model_trainer.setDataDirectory(r"./megablock/")
model_trainer.trainModel(num_experiments=100, batch_size=32)