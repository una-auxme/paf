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
      Image-Segmentation:
      - deeplabv3_resnet101
      -->
    <param name="model" value="deeplabv3_resnet101" />
  </node>
`

Depending on preferences and targets a different model can be used by replacing the value of the model parameter
by one of the lines from the comment above.

The Vision-Node will automatically switch between object-detection, imagesegmentation, load the correct weights and perform the correct preprocessing.

For now the Vision-Node only supports pyTorch models. Within the next sprint it should be able to
accept other frameworks aswell. It should also be possible to run object-detection and image-segmentation at the same time.

## How it works

### Initialization

The Vision-Node contains a Dictionary with all it's models. Depending on the model parameter it will initialize the correct model and weights.

`
self.model_dict = {
            "fasterrcnn_resnet50_fpn_v2": (fasterrcnn_resnet50_fpn_v2(weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT), FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT, "detection", "pyTorch"),
            "fasterrcnn_mobilenet_v3_large_320_fpn": (fasterrcnn_mobilenet_v3_large_320_fpn(weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT), FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT, "detection", "pyTorch"),
            "deeplabv3_resnet101": (deeplabv3_resnet101(weights=DeepLabV3_ResNet101_Weights.DEFAULT), DeepLabV3_ResNet101_Weights.DEFAULT, "segmentation", "pyTorch")
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

The Vision-Node implements an ImagePublisher under the topic: "/paf//Center/segmented_image"

The Configuartion File of RViz has been changed accordingly to display the published images alongside with the Camera.

## Known Issues

### Time

First experiments showed that the handle_camera_image function is way to slow to be used reliably. It takes around 1.5 seconds to handle one image.

Right now the Vision-Node is not using cuda due to cuda-memory-issues that couldn't be fixed right away.

The performance is expected to rise quite a bit when using cuda.

Also their is lots more room for testing different models inside the Vision-Node to evualte their accuracy and time-performance.
