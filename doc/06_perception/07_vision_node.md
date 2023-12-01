# Vision Node

The Visison Node serves as a replacement for the previous segmentation-node.
It provides an adaptive interface that is able to perform object-detection or image-segmentation
on several different models. The model can be specified as a parameter in the perception.launch file.

## Usage

The following code shows how the Vision-Node is specified in perception.launch

`
<node pkg="perception" type="vision_node.py" name="VisionNode" output="screen">
    <param name="role_name" value="$(arg role_name)" />
    <param name="side" value="Center" />
     <!--
      Object-Detection:
      - fasterrcnn_resnet50_fpn_v2
      - fasterrcnn_mobilenet_v3_large_320_fpn
      - yolov8n
      - yolov8s
      - yolov8m
      - yolov8l
      - yolov8x
      - yolo_nas_l
      - yolo_nas_m
      - yolo_nas_s
      - rtdetr-l
      - rtdetr-x
      - sam_l
      - FastSAM-x
      Image-Segmentation:
      - deeplabv3_resnet101
      - yolov8x-seg
      -->
    <param name="model" value="yolov8x-seg" />
  </node>
`

Depending on preferences and targets a different model can be used by replacing the value of the model parameter
by one of the lines from the comment above.

The Vision-Node will automatically switch between object-detection, imagesegmentation, load the correct weights and perform the correct preprocessing.

For now the Vision-Node only supports pyTorch models. Within the next sprint it should be able to
accept other frameworks aswell. It should also be possible to run object-detection and image-segmentation at the same time.

## Model overview

| Model                                 | Type         | Stable | Comments                              |
|---------------------------------------|--------------|--------|---------------------------------------|
| fasterrcnn_resnet50_fpn_v2            | detection    | no     | CUDA-Problems                         |
| fasterrcnn_mobilenet_v3_large_320_fpn | detection    | no     | CUDA-Problems                         |
| yolov8n                               | detection    | yes    |                                       |
| yolov8s                               | detection    | yes    |                                       |
| yolov8m                               | detection    | yes    |                                       |
| yolov8l                               | detection    | yes    |                                       |
| yolov8x                               | detection    | yes    |                                       |
| yolo_nas_l                            | detection    | no     | Missing super_gradients package error |
| yolo_nas_m                            | detection    | no     | Missing super_gradients package error |
| yolo_nas_s                            | detection    | no     | Missing super_gradients package error |
| rtdetr-l                              | detection    | yes    |                                       |
| rtdetr-x                              | detection    | yes    |                                       |
| sam_l                                 | detection    | no     | Ultralytics Error                     |
| FastSAM-x                             | detection    | no     | CUDA Problems                         |
| deeplabv3_resnet101                   | segmentation | no     | CUDA Problems, Segmentation Problems  |
| yolov8x-seg                           | segmentation | yes    |                                       |

## How it works

### Initialization

The Vision-Node contains a Dictionary with all it's models. Depending on the model parameter it will initialize the correct model and weights.

`
self.model_dict = {
            "fasterrcnn_resnet50_fpn_v2":
            (fasterrcnn_resnet50_fpn_v2(
                weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT),
                FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT,
                "detection",
                "pyTorch"),
            "fasterrcnn_mobilenet_v3_large_320_fpn":
            (fasterrcnn_mobilenet_v3_large_320_fpn(
                weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT),
                FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT,
                "detection",
                "pyTorch"),
            "deeplabv3_resnet101":
            (deeplabv3_resnet101(
                weights=DeepLabV3_ResNet101_Weights.DEFAULT),
                DeepLabV3_ResNet101_Weights.DEFAULT,
                "segmentation",
                "pyTorch"),
            'yolov8n': (YOLO, "yolov8n.pt", "detection", "ultralytics"),
            'yolov8s': (YOLO, "yolov8s.pt", "detection", "ultralytics"),
            'yolov8m': (YOLO, "yolov8m.pt", "detection", "ultralytics"),
            'yolov8l': (YOLO, "yolov8l.pt", "detection", "ultralytics"),
            'yolov8x': (YOLO, "yolov8x.pt", "detection", "ultralytics"),
            'yolo_nas_l': (NAS, "yolo_nas_l.pt", "detection", "ultralytics"),
            'yolo_nas_m': (NAS, "yolo_nas_m.pt", "detection", "ultralytics"),
            'yolo_nas_s': (NAS, "yolo_nas_s.pt", "detection", "ultralytics"),
            'rtdetr-l': (RTDETR, "rtdetr-l.pt", "detection", "ultralytics"),
            'rtdetr-x': (RTDETR, "rtdetr-x.pt", "detection", "ultralytics"),
            'yolov8x-seg': (YOLO, "yolov8x-seg.pt", "segmentation", "ultralytics"),
            'sam_l': (SAM, "sam_l.pt", "detection", "ultralytics"),
            'FastSAM-x': (FastSAM, "FastSAM-x.pt", "detection", "ultralytics"),

        }
`

### Core

The core of the Vision-Node is the handle_camera_image function.
This function is automatically triggered by the Camera-Subscriber of the Vision-Node and performs the following steps:

1. Convert ImageMsg to CV2-Image
2. Perform preprocessing on CV2-Image
3. Forward image through model
4. Call further processing function for output depending on type
   1. Detection -> apply_bounding_boxes
   2. Segmentation -> create_mask
5. Convert CV2-Image to ImageMsg
6. Publish ImageMsg over ImagePublisher

## Visualization

The Vision-Node implements an ImagePublisher under the topic: "/paf/hero/Center/segmented_image"

The Configuration File of RViz has been changed accordingly to display the published images alongside with the Camera.

The build in Visualization of the YOLO-Models works very well.

## Known Issues

### Time

When running on YOLO-Models the Time issue is fixed because ultralytics has some way of managing the CUDA-Resources very well.

When running on different models, the CUDA-Error persists.

## Segmentation

For some reason the create_segmentation mask function works in a standalone project, but not in the Vision-Node.
I stopped debugging, because the YOLO-Models work way better and build a very good and stable baseline.
