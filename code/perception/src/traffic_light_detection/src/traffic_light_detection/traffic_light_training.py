import torch.cuda
import torchvision.transforms as t
from tqdm import tqdm
from torch.optim.adam import Adam
from torch.optim.lr_scheduler import ExponentialLR
from torch.utils.data import DataLoader
from torchvision.datasets import ImageFolder
from torchvision.transforms import ToTensor
from dvclive import Live
from ruamel.yaml import YAML
import sys
import os
sys.path.append(os.path.abspath(sys.path[0] + '/..'))
from traffic_light_detection.transforms import Normalize, ResizeAndPadToSquare, \
    load_image  # noqa: E402
from data_generation.weights_organizer import WeightsOrganizer  # noqa: E402
from traffic_light_detection.classification_model import ClassificationModel \
    # noqa: E402
from traffic_light_config import TrafficLightConfig  # noqa: E402


class TrafficLightTraining:

    def __init__(self, cfg):
        """
        Initializes an instance to train a traffic light classification model.
        @param cfg: Config file for traffic light classification
        """
        self.cfg = cfg
        train_transforms = t.Compose([
            ToTensor(),
            ResizeAndPadToSquare([32, 32]),
            Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
            # ApplyMask(dataset_root + "/mask.png")
        ])
        val_transforms = t.Compose([
            ToTensor(),
            ResizeAndPadToSquare([32, 32]),
            Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        self.train_dataset = ImageFolder(root=self.cfg.DATASET_PATH + "/train",
                                         transform=train_transforms,
                                         loader=load_image)
        self.train_loader = DataLoader(dataset=self.train_dataset,
                                       batch_size=self.cfg.BATCH_SIZE,
                                       num_workers=self.cfg.NUM_WORKERS,
                                       shuffle=True)
        self.val_dataset = ImageFolder(root=self.cfg.DATASET_PATH + "/val",
                                       transform=val_transforms,
                                       loader=load_image)
        self.val_loader = DataLoader(dataset=self.val_dataset,
                                     batch_size=self.cfg.BATCH_SIZE,
                                     num_workers=self.cfg.NUM_WORKERS)
        self.model = ClassificationModel(num_classes=self.cfg.NUM_CLASSES,
                                         in_channels=self.cfg.NUM_CHANNELS)
        self.model = self.model.to(self.cfg.DEVICE)
        self.optimizer = Adam(self.model.parameters())
        self.lr_scheduler = ExponentialLR(self.optimizer, 0.95)
        self.loss_function = torch.nn.CrossEntropyLoss()
        self.weights_organizer = WeightsOrganizer(cfg=self.cfg,
                                                  model=self.model)

        self.live = Live()

    def run(self):
        """
        Trains the model for a given amount of epochs
        """
        tepoch = tqdm(range(self.cfg.EPOCHS))
        for i in range(self.cfg.EPOCHS):
            epoch_loss, epoch_correct = self.epoch()
            self.live.log_metric("train/accuracy", epoch_correct)
            self.live.log_metric("train/loss", epoch_loss)
            loss, correct = self.validate()
            self.live.log_metric("validation/accuracy", correct)
            self.live.log_metric("validation/loss", loss)
            self.lr_scheduler.step()
            self.live.next_step()
            tepoch.set_postfix(loss=epoch_loss, accuracy=epoch_correct,
                               val_loss=loss, val_accuracy=correct)
            tepoch.update(1)
            print(tepoch)
            self.weights_organizer.save(epoch_correct, correct)

    def epoch(self):
        """
        Trains the model for a single epoch
        @return: Average loss and accuracy of the net in this epoch
        """
        self.model.train()
        epoch_loss = 0
        epoch_correct = 0
        for i, data in enumerate(self.train_loader):
            images = data[0].to(self.cfg.DEVICE)
            labels = data[1].to(self.cfg.DEVICE)
            self.optimizer.zero_grad()

            outputs = self.model(images)
            loss = self.loss_function(outputs, labels)
            epoch_loss += loss.item()
            _, predictions = torch.max(outputs.data, 1)
            corr = (predictions == labels)
            epoch_correct += corr.sum().item()

            loss.backward()
            self.optimizer.step()
        epoch_loss /= len(self.train_dataset)
        epoch_correct /= len(self.train_dataset)
        return epoch_loss, 100. * epoch_correct

    def validate(self):
        """
        Executes the model validation on the validation-subset.
        @return: Average loss and accuracy of the net on the validation-subset
        """
        self.model.eval()
        val_loss = 0.
        val_correct = 0
        for i, data in enumerate(self.val_loader):
            images = data[0].to(self.cfg.DEVICE)
            labels = data[1].to(self.cfg.DEVICE)

            with torch.no_grad():
                outputs = self.model(images)
            loss = self.loss_function(outputs, labels)
            val_loss += loss.item()
            _, predictions = torch.max(outputs.data, 1)
            corr = (predictions == labels)
            val_correct += corr.sum().item()

        val_loss /= len(self.val_dataset)
        val_correct /= len(self.val_dataset)
        return val_loss, 100. * val_correct


if __name__ == '__main__':
    yaml = YAML(typ="safe")
    with open("params.yaml") as f:
        params = yaml.load(f)

    cfg = TrafficLightConfig()
    cfg.EPOCHS = params['train']['epochs']
    cfg.BATCH_SIZE = params['train']['batch_size']
    print(f"Computation device: {cfg.DEVICE}\n")
    tr = TrafficLightTraining(cfg)
    tr.run()
