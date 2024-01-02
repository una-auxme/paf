import torch
import torch.nn as nn
import torch.nn.functional as F


class ClassificationModel(nn.Module):

    def __init__(self, num_classes, in_channels=3):
        """
        Creates a small classification net for traffic light
        and traffic sign classification.
        @param num_classes: Number of classes
        """
        super(ClassificationModel, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=in_channels, out_channels=4,
                               kernel_size=5, padding='same')
        self.batch_norm1 = nn.BatchNorm2d(num_features=4)
        self.conv2 = nn.Conv2d(in_channels=4, out_channels=4, kernel_size=5,
                               padding='same')
        self.max_pool1 = nn.MaxPool2d(kernel_size=(2, 2))
        self.conv3 = nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3,
                               padding='same')
        self.max_pool2 = nn.MaxPool2d(kernel_size=(2, 2))
        self.conv4 = nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3,
                               padding='same')
        self.max_pool3 = nn.MaxPool2d(kernel_size=(2, 2))
        self.flatten = nn.Flatten()
        self.dropout = nn.Dropout(p=0.3)
        self.linear = nn.Linear(in_features=64, out_features=num_classes)

    def forward(self, x):
        """
        Propagates an image through the classification net.

        @param x: input images of shape (B, C, H, W)
        @return: classification scores  of shape (B, Classes) for input image
        """
        x = F.relu(self.conv1(x))
        x = self.batch_norm1(x)
        x = F.relu(self.conv2(x))
        x = self.max_pool1(x)
        x = F.relu(self.conv3(x))
        x = self.max_pool2(x)
        x = F.relu(self.conv4(x))
        x = self.max_pool3(x)
        x = self.flatten(x)
        x = self.dropout(x)
        x = F.softmax(self.linear(x), dim=1)
        return x

    @staticmethod
    def load_model(cfg):
        """
        Loads a pretrained model by given path in config file.
        @return: object of type ClassificationModel with pretrained weights
        """
        model = ClassificationModel(cfg.NUM_CLASSES, cfg.NUM_CHANNELS)
        path = cfg.MODEL_PATH
        if path is not None:
            try:
                state_dict = torch.load(path)
                model.load_state_dict(state_dict)
                print(f"Pretrained model loaded from {path}")
                return model
            except Exception as e:
                print(f"No pretrained model found at {path}: {e}\n"
                      f"Created new model with random weights.")
        return model.eval()
