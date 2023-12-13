'''
Docs: https://www.tensorflow.org/hub/tutorials/tf2_object_detection, https://pylot.readthedocs.io/en/latest/perception.detection.html
'''

from globals import IMAGE_BASE_FOLDER, IMAGES_FOR_TEST

import os
from time import perf_counter
import tensorflow as tf
import tensorflow_hub as hub

import numpy as np
from six import BytesIO
from PIL import Image

from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt

from object_detection.utils import visualization_utils as viz_utils

matplotlib.use('TkAgg')

tf.get_logger().setLevel('ERROR')

ALL_MODELS = [
    'faster-rcnn',
    'ssdlite-mobilenet-v2',
    'ssd-mobilenet-fpn-640',
    'ssd-mobilenet-v1',
    'ssd-mobilenet-v1-fpn'
]

MODEL_BASE_FOLDER = '/home/maxi/Downloads/models/obstacle_detection'

LABEL_FILE = '/home/maxi/Downloads/pylot.names'


def load_image_into_numpy_array(path):
    image_data = tf.io.gfile.GFile(path, 'rb').read()
    image = Image.open(BytesIO(image_data))

    (im_width, im_height) = image.size
    return np.array(image.convert('RGB').getdata()).reshape(
        (1, im_height, im_width, 3)).astype(np.uint8)


def load_model(model_name):
    model_handle = os.path.join(MODEL_BASE_FOLDER, model_name)

    print('Selected model: ' + model_name)

    print('Loading model...', end='')
    hub_model = hub.load(model_handle)
    print(' done!')

    return hub_model


def get_category_index(label_file):
    with open(label_file, 'r') as f:
        labels = f.readlines()
        labels = [label.strip() for label in labels]
        category_index = {i: {'id': i, 'name': name} for i, name in enumerate(labels)}
    return category_index


if not os.path.exists(f'{IMAGE_BASE_FOLDER}/result'):
    os.makedirs(f'{IMAGE_BASE_FOLDER}/result')

category_index = get_category_index(LABEL_FILE)

for m in ALL_MODELS:
    model = load_model(m)
    first_gen = True

    for p in IMAGES_FOR_TEST:
        image_path = os.path.join(IMAGE_BASE_FOLDER, IMAGES_FOR_TEST[p])
        image_np = load_image_into_numpy_array(image_path)
        image_tensor = tf.convert_to_tensor(image_np)

        if first_gen:
            print('Running warmup inference...')
            model.signatures['serving_default'](image_tensor)
            first_gen = False

        print(f'Running inference for {p}... ')

        start_time = perf_counter()

        # running inference
        results = model.signatures['serving_default'](image_tensor)

        elapsed_time = perf_counter() - start_time

        # different object detection models have additional results
        # all of them are explained in the documentation
        result = {key: value.numpy() for key, value in results.items()}

        label_id_offset = -1
        image_np_with_detections = image_np.copy()

        viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections[0],
            result['boxes'][0],
            (result['classes'][0] + label_id_offset).astype(int),
            result['scores'][0],
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=.10,
            agnostic_mode=False)

        file_name = Path(image_path).stem

        plt.figure(figsize=(32, 18))
        plt.title(f'Pylot (TF) - {m} - {p} - {elapsed_time*1000:.0f}ms', fontsize=30)
        plt.imshow(image_np_with_detections[0])
        plt.savefig(f'{IMAGE_BASE_FOLDER}/result/{file_name}_TF_{m}.jpg')
        plt.close()
