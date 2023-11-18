'''
Docs: https://docs.ultralytics.com/modes/predict/, https://docs.ultralytics.com/tasks/detect/#models
'''

import os
from globals import IMAGE_BASE_FOLDER, IMAGES_FOR_TEST
from ultralytics import YOLO
from PIL import Image

ALL_MODELS = [
    'yolov8n',
    'yolov8s',
    'yolov8m',
    'yolov8l',
    'yolov8x',
]

for m in ALL_MODELS:
    print('Selected model: ' + m)
    model_path = os.path.join('yolo', m + '.pt')
    model = YOLO(model_path)

    for p in IMAGES_FOR_TEST:
        image_path = os.path.join(IMAGE_BASE_FOLDER, IMAGES_FOR_TEST[p])
        img = Image.open(image_path)

        results = model.predict(source=img, save=True, save_conf=True, line_width=1, half=True)

    del model
