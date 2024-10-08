# Dataset structure

**Summary:** This document gives a short overview about the structure of our dataset that is needed to train EfficientPS.

---

## Author

Marco Riedenauer

## Date

19.02.2023

<!-- TOC -->
- [Dataset structure](#dataset-structure)
  - [Author](#author)
  - [Date](#date)
  - [Converting the dataset](#converting-the-dataset)
  - [Preparation of the dataset for training](#preparation-of-the-dataset-for-training)
    - [Explanation of the conversion of groundtruth images](#explanation-of-the-conversion-of-groundtruth-images)
      - [Things](#things)
      - [Stuff](#stuff)
    - [Explanation of creating json files](#explanation-of-creating-json-files)
<!-- TOC -->

## Converting the dataset

After creating the dataset with the [Dataset Generator](01_dataset_generator.md) or creating a dataset on your own,
execute the [Dataset Converter](../../code/perception/src/dataset_converter.py) to ensure that your dataset has the
following structure:

```diff
.
├── groundtruth
│   ├── test
│   │   ├── <city_id_1>
│   │   │   ├── <city_id_1>_<camera_name_x>_<image_number_y>_groundtruth.png
│   │   │   └── ...
│   │   ├── <city_id_2>
│   │   │   ├── <city_id_2>_<camera_name_x>_<image_number_y>_groundtruth.png
│   │   │   └── ...
│   │   └── ...
│   ├── train
│   │   └── ...
│   └── val
│       └── ...
├── <camera_name1>
│   ├── test
│   │   └── <city_id_1>
│   │       ├── <city_id_1>_<camera_name_x>_<image_number_y>.png
│   │       └── ...
│   ├── train
│   │   └── ...
│   └── val
│       └── ...
├── <camera_name_2>
│   └── ...
└── ...
```

## Preparation of the dataset for training

When the dataset has the correct structure, the groundtruth images have to be converted to COCO format and some
json files have to be created.

To do so, execute the following command in an attached shell:

```shell
python3 perception/src/panoptic_segmentation/preparation/createPanopticImgs.py --dataset_folder <path_to_dataset_root>
```

This will create the json files in the groundtruth folder and for each groundtruth image the respective converted
image, which will be saved in the same place as the original image. The original images from CARLA aren't needed
anymore then.

### Explanation of the conversion of groundtruth images

The output of the instance segmentation camera from CARLA is an RGB image with the classId saved in the red channel and
the instanceId saved in the green and blue channels.

During training, the ground truth images must be available as COCO formatted images, which are two-dimensional arrays
with one channel each. In this channel, the corresponding class and instance ID are stored for each pixel as follows:

#### Things

  ```markdown
  value = classId * 1000 + instanceId
  ```
  
  This means, for example, that all pixels of the tenth car in an image have the value 14009.
  The class car has the ID 14 and the instance ID of the tenth car is 9.

#### Stuff

  ```markdown
  value = classId
  ```
  
  Since stuff has no instances, the instanceId is not needed here.

### Explanation of creating json files

To save execution time during training, the image paths, labels and corresponding bounding boxes are saved in separate
files for train, test and validation subsets.
These files have the following structure:

```jsonpath

annotation{
    "file_name"       : str, (e.g. "2_back_0_groundtruth.png")
    "image_id"        : str, (e.g. "2_back_0")
    "segments_info"   : [segment_info],
}

categories[{
    "color"           : [R,G,B],
    "id"              : int,
    "isthing"         : 0 or 1,
    "name"            : str,
    "supercategory"   : str
}]

images[{
    "file_name"       : str (e.g. "2_back_0.png"),
    "height"          : int
    "id"              : str (e.g. "2_back_0")
    "width"           : int
}]

```

whereby ```segment_info``` is defined as:

```jsonpath
segment_info{
          "area"            : int,
          "bbox"            : [x,y,width,height],
          "category_id"     : int,
          "id"              : int,
          "iscrowd"         : 0 or 1,
      }
```
