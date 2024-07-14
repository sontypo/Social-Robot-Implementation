# YOLOv8 Model Training Package
This package provides essential commands for training a YOLOv8 model to recognize objects.
## Preparing a custom dataset for YOLOv8
- Generate datasets with [Roboflow](https://app.roboflow.com/) and download this to ``data``
- Trainning
### YOLOv8 Instance Segmentation
To perform instance segmentation with YOLOv8, execute the following commands:
```
python3 train_seg.py 
```
Validate the data using the image named "test_predict.jpg":
```
python3 predict_seg.py
```
After that, go to ``runs/segment/predict`` to check the model

### YOLOv8 Object detection
For YOLOv8 object detection, use the following commands:
```
python3 train.py 
```
Validate the data using the image named "test_predict.jpg":
```
python3 predict.py
```
After that, go to ``runs/detec/predict`` to check the model
