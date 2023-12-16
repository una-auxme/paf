import argparse

import torch.cuda
import torchvision.transforms as t
from traffic_light_detection.src.traffic_light_detection.transforms import Normalize, ResizeAndPadToSquare, \
    load_image
from traffic_light_detection.src.traffic_light_detection.classification_model import ClassificationModel
from torchvision.transforms import ToTensor
from traffic_light_detection.src.traffic_light_config import TrafficLightConfig


def parse_args():
    """
    Parses arguments for execution given by the command line.
    @return: Parsed arguments
    """
    parser = argparse.ArgumentParser(description='Inference traffic light '
                                                 'detection')
    parser.add_argument('--model',
                        default='/opt/project/code/perception/src/'
                                'traffic_light_detection/models/'
                                '05.12.2022_17.47/'
                                'model_acc_99.53_val_100.0.pt',
                        help='path to pretrained model',
                        type=str)
    parser.add_argument('--image', default=None,
                        help='/dataset/val/green/green_83.png',
                        type=str)
    return parser.parse_args()


class TrafficLightInference:

    def __init__(self, model_path):
        """
        Initializes an instance to inference a TLD net.
        @param model_path: Path to a pretrained model to be used as classifier
        """
        self.cfg = TrafficLightConfig()
        self.cfg.MODEL_PATH = model_path
        self.transforms = t.Compose([
            ToTensor(),
            ResizeAndPadToSquare([32, 32]),
            Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])

        self.model = ClassificationModel.load_model(self.cfg)
        self.model = self.model.to(self.cfg.DEVICE)
        self.class_dict = {0: 'Backside',
                           1: 'Green',
                           2: 'Red',
                           3: 'Side',
                           4: 'Yellow'}

    def __call__(self, img):
        """
        Classifies an image
        """
        img = self.transforms(img)
        img = img.to(self.cfg.DEVICE)
        if len(img.shape) == 3:
            out = self.model(img[None, :])
        else:
            out = self.model(img)
        _, prediction = torch.max(out.data, 1)
        return prediction.item()


# main function for testing purposes
if __name__ == '__main__':
    args = parse_args()
    image_path = args.image
    image = load_image(image_path)
    classifier = TrafficLightInference(args.model)
    pred = classifier(image)
    print(f"Classification: {pred} ({classifier.class_dict[pred]})")
