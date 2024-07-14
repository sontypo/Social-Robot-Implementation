from ultralytics import YOLO
import os

dirPath = os.path.dirname(os.path.realpath(__file__))

print(dirPath)

# Load a model
model = YOLO('yolov8n-seg.yaml')  # build a new model from YAML
model = YOLO('yolov8n-seg.pt')  # load a pretrained model (recommended for training)
model = YOLO('yolov8n-seg.yaml').load('yolov8n.pt')  # build from YAML and transfer weights

# Train the model
results = model.train(data=dirPath+'/data/data.yaml', epochs=150, batch=16, imgsz=640)
