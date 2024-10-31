# VisionNode Code Summary

The `VisionNode` class is designed to perform object detection and segmentation tasks using both PyTorch and Ultralytics models. It is structured to publish detection and segmentation results in ROS.

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Overview](#overview)
- [Class Initialization](#class-initialization)
- [Setup Functions](#setup-functions)
  - [Camera and Distance Array Subscriptions](#camera-and-distance-array-subscriptions)
  - [Camera Publishers](#camera-publishers)
  - [Object Distance Publishers](#object-distance-publishers)
  - [Traffic Light Publishers](#traffic-light-publishers)
- [Main Processing Functions](#main-processing-functions)
  - [Handling Camera Images](#handling-camera-images)
  - [Distance Array Handling](#distance-array-handling)
  - [Prediction with PyTorch](#prediction-with-pytorch)
  - [Prediction with Ultralytics](#prediction-with-ultralytics)
  - [Traffic Light Processing](#traffic-light-processing)
  - [Bounding Box and Segmentation Mask Creation](#bounding-box-and-segmentation-mask-creation)
- [Utility Functions](#utility-functions)
  - [Minimum X and Y Calculations](#minimum-x-and-y-calculations)
- [Models](#models)


## Overview
The `VisionNode` class leverages both PyTorch and Ultralytics pretrained models for object detection and segmentation, integrating features such as bounding boxes, segmentation masks, and distance calculations based on LIDAR data.

## Class Initialization
Upon instantiation, `VisionNode` initializes various configurations, such as:
- **Model Dictionary** (`self.model_dict`): Holds model configurations with different detection and segmentation models from PyTorch and Ultralytics.
- **Camera Configurations**: Subscribes to topics for front, rear, left, and right camera views if specified.
- **Device Selection**: Automatically selects `CUDA` if available.
- **Publishers and Subscribers**: Sets up publishers and subscribers for camera images, distance arrays, and traffic light detections.
- **Model Setup**: Loads and initializes the model specified in ROS parameters (`self.get_param("model")`) and moves it to the selected device.

## Setup Functions

### Camera and Distance Array Subscriptions
These functions subscribe to the camera topics, allowing the node to receive images from multiple camera angles and perform image processing tasks.

- **`setup_camera_subscriptions`**: Subscribes to specified camera angles.
- **`setup_dist_array_subscription`**: Subscribes to LIDAR depth data.

### Camera Publishers
**`setup_camera_publishers`** sets up publishers for each camera angle. Each angle has a unique topic to publish segmented images back to ROS.

### Object Distance Publishers
**`setup_object_distance_publishers`** creates a publisher that outputs a list of detected objects and their distances.

### Traffic Light Publishers
**`setup_traffic_light_publishers`** sets up a publisher for detected traffic light images.

## Main Processing Functions

### Handling Camera Images
**`handle_camera_image`** receives an image message, selects the appropriate prediction framework (PyTorch or Ultralytics), and processes the image. After processing, it publishes the result on the appropriate topic based on the camera angle.

### Distance Array Handling
**`handle_dist_array`** receives LIDAR data and updates the `self.dist_arrays` attribute with the latest depth image data.

### Prediction with PyTorch
**`predict_torch`** preprocesses images, applies the PyTorch model, and draws either bounding boxes or segmentation masks based on the model type (detection or segmentation).

### Prediction with Ultralytics
**`predict_ultralytics`** preprocesses images and applies an Ultralytics model, handling object detection and drawing bounding boxes on the image. This function includes distance calculation logic using LIDAR data within the bounding boxes.

### Traffic Light Processing
**`process_traffic_lights`** identifies traffic lights based on object detection outputs. It crops the detected regions and publishes segmented traffic light images aswell as distance information to a separate topic.

### Bounding Box and Segmentation Mask Creation
- **`create_mask`**: Generates a segmentation mask for PyTorch segmentation models.
- **`apply_bounding_boxes`**: Draws bounding boxes on detection models and labels each bounding box with its respective object type and distance information.

## Utility Functions

### Minimum X and Y Calculations
- **`min_x`**: Calculates the minimum x-distance for objects in LIDAR data, representing the closest object in the forward direction.
- **`min_abs_y`**: Calculates the minimum y-distance  in absolute terms, representing the closest object sideways.


## Models

Following there will be a short overview of the used machine learning and computer vision models. 

| Model                            | Techniques                        | Features                                                                                  | Description                                                                                        |
| -------------------------------- | --------------------------------- | ----------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| frcnn_resnet50_fpn_v2            | Faster R-CNN, ResNet, FPN         | High accuracy and computational power                                                     | Object detection with Region Proposal and Feature Pyramid Network (FPN) for multi-scale detection. |
| frcnn_mobilenet_v3_large_320_fpn | Faster R-CNN, MobileNet, FPN      | For less computationally intensive tasks and mobile applications.                         | Compact model with Region Proposal and FPN for efficient object detection.                         |
| deeplabv3_resnet101              | DeepLabV3, ResNet                 | Specializes in image segmentation, uses ResNet101 backbone for strong feature extraction. | Pixel-level segmentation with Atrous convolutions for contextual information.                      |
| yolov8n                          | YOLOv8 Nano                       | Small, fast, low computational demand, lower accuracy.                                    | Real-time object detection, fast processing.                                                       |
| yolov8s                          | YOLOv8 Small                      | Small, fast, low resource consumption.                                                    | Real-time object detection, fast processing.                                                       |
| yolov8m                          | YOLOv8 Medium                     | Balanced performance and precision.                                                       | Real-time object detection, fast processing.                                                       |
| yolov8l                          | YOLOv8 Large                      | Larger, requires more resources, higher accuracy.                                         | Real-time object detection, fast processing.                                                       |
| yolov8x                          | YOLOv8 Extra-Large                | Largest variant, best precision, higher computational load.                               | Real-time object detection, fast processing.                                                       |
| yolo_nas_l                       | YOLO, NAS Large                   | Automatically optimized architecture for larger hardware resources.                       | Optimized through automated architecture search for specific hardware requirements.                |
| yolo_nas_m                       | YOLO, NAS Medium                  | Optimized architecture, medium hardware requirements.                                     | Optimized through automated architecture search for specific hardware requirements.                |
| yolo_nas_s                       | YOLO, NAS Small                   | Most compact architecture for resource-saving applications.                               | Optimized through automated architecture search for specific hardware requirements.                |
| rtdetr-l                         | RT-DETR (Transformer) Large       | Transformer model, real-time capability, medium accuracy.                                 | Transformer-based real-time object detection.                                                      |
| rtdetr-x                         | RT-DETR (Transformer) Extra Large | Transformer model, higher accuracy, higher computational load.                            | Transformer-based real-time object detection.                                                      |
| yolov8x-seg                      | YOLO (Segmentation)               | Only detection without segmentation.                                                      | Extension of YOLO for pixel-precise object detection.                                              |
| sam_l                            | SAM                               | High accuracy in segmentation for universal applications.                                 | Universal segmentation using SAM technique.                                                        |
| FastSAM-x                        | Fast SAM                          | Faster variant for real-time application requirements.                                    | Fast segmentation using an accelerated SAM model.                                                  |

