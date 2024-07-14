from ultralytics import YOLO
import os
import cv2

dirPath = os.path.dirname(os.path.realpath(__file__))
print(dirPath)

# Load a model
model = YOLO('yolov8n.pt')  # load an official model
model = YOLO(dirPath+'/runs/detect/train/weights/last.pt')  # load a custom trained
model.predict('test_predict.jpg', save=True)

