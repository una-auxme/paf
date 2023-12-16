from datetime import datetime
from pathlib import Path
import torch


class TrafficLightConfig:

    def __init__(self):
        # General settings
        self.DEVICE = ('cuda' if torch.cuda.is_available() else 'cpu')
        self.TIME = datetime.now().strftime("%d.%m.%Y_%H.%M")

        # Training
        self.ROOT_PATH = str(Path(__file__).resolve().parents[1].resolve())
        self.DATASET_PATH = self.ROOT_PATH + "/dataset"
        self.WEIGHTS_PATH = self.ROOT_PATH + "/models/"
        # Amount of epochs to train
        # One epoch: Training with all images from training dataset once
        self.NUM_WORKERS = 4
        self.NUM_CLASSES = 5  # Traffic light states: green, yellow, red, back, side
        self.NUM_CHANNELS = 3  # RGB encoded images

        # Inference
        self.MODEL_PATH = None
