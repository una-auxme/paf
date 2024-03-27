# Evaluation of the PAF22 Traffic Light Detection

In this experiment, the existing Traffic Light Detection from PAF22 has been tested.
The goals was to be able to verify, that it is suitable for PAF23.

## Model

The architecture of the model is a Convolutional Neural Network (CNN) and it consists of the following layers:

1. **Convolutional Layer 1**: This layer uses a 2D convolution over an input signal composed of several input planes, with in_channels input channels, 4 output channels, a kernel size of 5, and padding set to 'same'. This means the output size is the same as the input size.
2. **Batch Normalization Layer**: This layer applies Batch Normalization over a 4D input (a mini-batch of 2D inputs with additional channel dimension) as described in the paper Batch Normalization: Accelerating Deep Network Training by Reducing Internal Covariate Shift.
3. **Convolutional Layer 2**: This layer is similar to the first convolutional layer but it takes the output of the first layer (4 channels) as input.
4. **Max Pooling Layer 1**: This layer uses a 2D max pooling over an input signal composed of several input planes, with a kernel size of (2, 2).
5. **Convolutional Layer 3**: This layer is similar to the previous convolutional layers but it has a kernel size of 3.
6. **Max Pooling Layer 2**: This layer is similar to the first max pooling layer.
7. **Convolutional Layer 4**: This layer is similar to the previous convolutional layers.
8. **Max Pooling Layer 3**: This layer is similar to the previous max pooling layers.
9. **Flatten Layer**: This layer flattens the input by removing the spatial dimensions.
10. **Dropout Layer**: This layer randomly zeroes some of the elements of the input tensor with probability p=0.3 using samples from a Bernoulli distribution.
11. **Linear Layer**: This layer applies a linear transformation to the incoming data. It has 64 input features and num_classes output features.

## Dataset

The existing dataset of PAF22 consists of 2340 images (combined) of the categories red, yellow, green, backside. There are also 382 validation images (combined).

The data can be accessed through DVC.

## Training

Running the training with `dvc exp run` in the traffic light detection directory, results in a trained model with >99% accuracy & validation.

## Examples

Result | Large | Small |
-----------|----------|----------|
Green | ![Green-Large](assets/green_4.png)  |  ![Green-Small](assets/green_22.jpg) |
Yellow | ![Yellow-Large](assets/yellow_1.png)  |  ![Yellow-Small](assets/yellow_18.jpg) |
Red | ![Red-Large](assets/red_10.png)  |  ![Red-Small](assets/red_20.png) |
Back | ![Back-Large](assets/back_1.png)  |  ![Back-Small](assets/back_14.jpg) |

## Verdict

The high accuracy and manual testing of the above example images verified, that the existing PAF22 traffic light detection model can be used for PAF23.
