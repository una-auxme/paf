'''
Docs: https://docs.ultralytics.com/modes/predict/, https://docs.ultralytics.com/tasks/detect/#models, https://docs.ultralytics.com/models/yolo-nas
'''

import os
from globals import IMAGE_BASE_FOLDER, IMAGES_FOR_TEST
from ultralytics import NAS, YOLO, RTDETR, SAM, FastSAM
from PIL import Image
import torch

ALL_MODELS = {
    'yolov8n': YOLO,
    'yolov8s': YOLO,
    'yolov8m': YOLO,
    'yolov8l': YOLO,
    'yolov8x': YOLO,
    'yolo_nas_l': NAS,
    'yolo_nas_m': NAS,
    'yolo_nas_s': NAS,
    'rtdetr-l': RTDETR,
    'rtdetr-x': RTDETR,
    'yolov8x-seg': YOLO,
    'sam-l': SAM,
    'FastSAM-x': FastSAM,
}


with torch.inference_mode():
    for m, wrapper in ALL_MODELS.items():
        print('Selected model: ' + m)
        model_path = os.path.join('yolo', m + '.pt')
        model = wrapper(model_path)

        for p in IMAGES_FOR_TEST:
            image_path = os.path.join(IMAGE_BASE_FOLDER, IMAGES_FOR_TEST[p])
            img = Image.open(image_path)

            _ = model.predict(source=img, save=True, save_conf=True, line_width=1, half=True)

        del model
