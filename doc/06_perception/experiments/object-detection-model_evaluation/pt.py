'''
Docs: https://pytorch.org/vision/stable/models.html#object-detection
'''

import os
from time import perf_counter
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import \
    FasterRCNN_MobileNet_V3_Large_320_FPN_Weights, \
    FasterRCNN_ResNet50_FPN_V2_Weights
from torchvision.models.detection.retinanet import \
    RetinaNet_ResNet50_FPN_V2_Weights
from globals import IMAGE_BASE_FOLDER, IMAGES_FOR_TEST
from torchvision.utils import draw_bounding_boxes
from pathlib import Path
import matplotlib.pyplot as plt
from PIL import Image
from torchvision import transforms
from torchvision.transforms.functional import to_pil_image

ALL_MODELS = {
    'fasterrcnn_mobilenet_v3_large_320_fpn':
    FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,
    'fasterrcnn_resnet50_fpn_v2': FasterRCNN_ResNet50_FPN_V2_Weights,
    'retinanet_resnet50_fpn_v2': RetinaNet_ResNet50_FPN_V2_Weights,
}


def load_model(model_name):
    print('Selected model: ' + model_name)
    print('Loading model...', end='')
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    weights = ALL_MODELS[model_name].DEFAULT
    model = torchvision.models.detection.__dict__[model_name](
            weights=weights
        ).to(device)
    model.eval()
    return model, weights, device


def load_image(image_path, model_weights, device):
    img = Image.open(image_path)
    img = img.convert('RGB')
    img = transforms.Compose([transforms.PILToTensor()])(img)
    img = model_weights.transforms()(img)
    img = img.unsqueeze_(0)
    img = img.to(device)

    return img


first_gen = True

for m in ALL_MODELS:
    model, weights, device = load_model(m)

    for p in IMAGES_FOR_TEST:
        image_path = os.path.join(IMAGE_BASE_FOLDER, IMAGES_FOR_TEST[p])
        image_np = load_image(image_path, weights, device)

        if first_gen:
            print('Running warmup inference...')
            model(image_np)
            first_gen = False

        print(f'Running inference for {p}... ')

        start_time = perf_counter()

        # running inference
        results = model(image_np)

        elapsed_time = perf_counter() - start_time

        # different object detection models have additional results
        # all of them are explained in the documentation
        result = results[0]

        label_id_offset = -1

        image_np_with_detections = torch.tensor(image_np * 255,
                                                dtype=torch.uint8)
        boxes = result['boxes']
        scores = result['scores']
        labels = [weights.meta["categories"][i] for i in result['labels']]

        box = draw_bounding_boxes(image_np_with_detections[0], boxes, labels,
                                  colors='red', width=2)
        box_img = to_pil_image(box)

        file_name = Path(image_path).stem

        plt.figure(figsize=(32, 18))
        plt.title(f'PyTorch - {m} - {p} - {elapsed_time*1000:.0f}ms',
                  fontsize=30)
        plt.imshow(box_img)
        plt.savefig(f'{IMAGE_BASE_FOLDER}/result/{file_name}_PT_{m}.jpg')
        plt.close()
