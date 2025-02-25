# Summary of Research for lane detection models

- [Overall](#overall)
- [Lane Detection on TuSimple](#lane-detection-on-tusimple)
- [Lane Detection on CULane](#lane-detection-on-culane)
- [Lane Detection on CurveLanes](#lane-detection-on-curvelanes)

## Overall

Lane Detection is a computer vision task that involves identifying the boundaries of driving lanes in a video or image of a road scene. The goal is to accurately locate and track the lane markings in real-time, even in challenging conditions such as poor lighting, glare, or complex road layouts.
[1]

The website PapersWithCode [1] gives several benchmarks for different lane detection models. In the following the best perfoming model of the three most common Datasets are presented. The models evaluation is based on the calculation of accuracy, precision, Recall, and F1-measure.
The formulas are presented in the image below. [3]

![Evaluation metric formulas for lane detection models](/assets/perception/Formula_for_Modelevaluation_lane_detection.png)

## Lane Detection on TuSimple

The TuSimple dataset consists of 6,408 road images on US highways. The resolution of image is 1280×720. The dataset is composed of 3,626 for training, 358 for validation, and 2,782 for testing called the TuSimple test set of which the images are under different weather conditions. [2]

The best performing model on TuSimple is called **SCNN_UNet_Attention_PL**. [3]

This study proposes a pipeline for lane detection through self
pre-training with masked sequential autoencoders and finetuning segmentation with customized PolyLoss. In the first
stage, the images are randomly masked as the inputs, and the
neural network model is pre-trained with reconstructing the
complete images as the objective. In the second stage, the pretrained neural network model weights are transferred to the
segmentation neural network model with the same backbone
and only the structure of the output layer is adjusted. In this
phase, continuous image frames without any masking are
served as inputs. The neural network weights are further
updated and fine-tuned by minimizing PolyLoss with the
backpropagation mechanism.

### SCNN_UNet_Attention_PL Advantages

- Robustness: The model is robust against difficult scenarios such as glare, shadows or obscured lines, which is ideal for simulating realistic road conditions.
  
- Training efficiency: Due to its fast convergence, it can be trained and implemented with moderate resource requirements.
  
- Continuous lane tracking: By processing multiple image sequences, the model provides reliable recognition results in dynamic driving scenarios.

### SCNN_UNet_Attention_PL Disadvantages

- No implementation or code found

## Lane Detection on CULane

CULane is a large scale challenging dataset for academic research on traffic lane detection. It is collected by cameras mounted on six different vehicles driven by different drivers in Beijing. More than 55 hours of videos were collected and 133,235 frames were extracted.
The dataset is divided into 88880 images for training set, 9675 for validation set, and 34680 for test set. The test set is divided into normal and 8 challenging categories. [4]

The best performing model on CULane is called **CLRerNet-DLA34**. [5]

The model uses the best-performing CLRNet for row-based lane representation. This means the lane instance is represented as a set of x-coordinates at the fixed rows. It combines this approach with a new metric, LaneIoU. LaneIoU considers local angle variations in the lane to improve accuracy
when comparing model predictions to actual lane positions.

### CLRerNet-DLA34 Advantages

- improved accuracy: With the LaneIoU metric, the model can detect tilted or curved lanes more precisely. In testing, CLRerNet achieved higher F1 scores than other models, especially in challenging conditions like curves or difficult lighting

### CLRerNet-DLA34 Disadvantages

- not perfect: Although CLRerNet shows significant improvement in performance, there still is a gap between
the best CLRerNet model’s performance (81.43%) and the
oracle-confidence case (98.47%)

## Lane Detection on CurveLanes

CurveLanes is a new benchmark lane detection dataset with 150K lanes images for difficult scenarios such as curves and multi-lanes in traffic lane detection. It is collected in real urban and highway scenarios in multiple cities in China.
It is the largest lane detection dataset so far and establishes a more challenging benchmark for the community. [6]

The best performing model on CurveLanes is **CondLSTR (ResNet-101)** [7]

### CondLSTR (ResNet-101)

The paper "Generating Dynamic Kernels via Transformers for Lane Detection" presents a novel approach to lane detection, using dynamic convolutional kernels generated by transformers.
This method achieves greater flexibility by adapting kernels to specific spatial features in images, which enhances detection accuracy in challenging environments.
The dynamic kernel approach allows the model to perform well in varied lane geometries and lighting conditions.
This framework demonstrates competitive performance on standard lane detection benchmarks, highlighting the potential of transformer-based architectures in dynamic feature extraction for computer vision tasks.

![Image-CondLSTR](../../../assets/perception/CondLSTR(RESNet101)_Model_Overview.jpg)

For more details, view the paper [here](https://openaccess.thecvf.com//content/ICCV2023/papers/Chen_Generating_Dynamic_Kernels_via_Transformers_for_Lane_Detection_ICCV_2023_paper.pdf).

**model explanation:**

- Input Encoding: The model begins by encoding the input lane images using a CNN backbone, capturing basic spatial features.
- Transformer Encoder: A transformer layer processes these features to capture long-range dependencies, which is particularly useful for lanes with varying curves and patterns.
- Dynamic Kernel Generation: Based on the encoded features, the model generates dynamic convolutional kernels. These kernels are adaptive, varying across spatial regions of the image, allowing the model to emphasize lane-relevant features more effectively.
- Convolutional Processing: The generated kernels are applied to process lane features dynamically, refining details at multiple levels, such as curves, visibility under different lighting, and occlusions.
- Output Layer: Finally, the model outputs lane detection maps, indicating lane positions on the input image.

**output:**

![Comparison of CondLSTR model performance](/assets/perception/Comparison_of_models_CondLSTR(RESNet101).jpg)

**advantages:**

- Dynamic Kernel Generation: The key strength of this model is its ability to generate dynamic convolutional kernels through transformers. This makes the model highly adaptable to varying lane structures, such as curves, occlusions, and intersections, compared to traditional static kernels​

- Handling Complex Lane Topologies: The transformer-based dynamic kernels capture global lane structure across the image, which is crucial for detecting lanes with complex patterns (e.g., forks, dense lanes) that static methods struggle with​

- Improved Robustness: Due to the dynamic nature of the kernels, the model performs better under challenging conditions like occlusions and varying lane configurations, which often confuse traditional methods​

**disadvantages:**

- Computational Complexity: While the model offers significant performance improvements, the use of transformers and dynamic kernel generation may result in higher computational cost and slower inference times, especially in real-time applications​

- Dependence on High-Quality Data: Like many deep learning models, this approach relies on large, well-labeled datasets for training. In environments where high-quality lane annotation is difficult or expensive to acquire, the model may struggle​

## Summary

| Model | Test Acc (%) | Precision | Recall | F1-Measure | Release | Dataset |
|--------|-------------|-----------|---------|------------|---------|----------|
| SCNN_UNet_Attention_PL | 98.36 | 0.937 | 0.911 | 0.924 | 2022 | TuSimple |
| CLRerNet-DLA34 | N/A | 0.917 | 0.818 | 0.814 | 2023 | CULane |
| CondLSTR(ResNet-101) | N/A | 0.913 | 0.858 | 0.885 | 2023 | CurveLanes |

![Comparison_on_dataset](/doc/assets/perception/Comparison_of_CurveLane_Dataset.jpg)

It should be mentioned that the paper by Robin Karlsson et. al was also read, but it was not worth comparing it here as it was a selfsupervised learning model and was not trained on the above datasets but on a smaller dataset and used more sensors and cameras
(6 cameras, 5 radars and 1 lidar) than allowed in the Qualify.
But for interrested people you can find the paper [here](https://arxiv.org/pdf/2304.13242v2)

## Decision

We chose to implement the CLRerNet-DLA34 and the CondLSTR(ResNet-101) because they have already implemented code bases to save time.

[1]: https://paperswithcode.com/task/lane-detection#datasets
[2]: https://paperswithcode.com/dataset/tusimple
[3]: https://arxiv.org/pdf/2305.17271v2
[4]: https://paperswithcode.com/dataset/culane
[5]: https://arxiv.org/pdf/2305.08366v1
[6]: https://paperswithcode.com/dataset/curvelanes
[7]: https://openaccess.thecvf.com//content/ICCV2023/papers/Chen_Generating_Dynamic_Kernels_via_Transformers_for_Lane_Detection_ICCV_2023_paper.pdf
