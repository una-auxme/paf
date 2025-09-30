import sys
import os
import torch


class WeightsOrganizer:

    def __init__(self, cfg, model):
        """
        Initializes a weights organizer for the given model.
        @param cfg: Config file for traffic light classification
        @param model: Classification model that has to be saved
        """
        self.cfg = cfg
        self.model = model
        self.best = []
        print(self.cfg.WEIGHTS_PATH)
        try:
            os.makedirs(self.cfg.WEIGHTS_PATH, exist_ok=True)
        except FileExistsError:
            sys.exit(
                f"The directory {self.cfg.WEIGHTS_PATH} already exists."
                f"Cannot create weights-directory for training."
                f"Try again in at least one minute."
            )

    def save(self, accuracy, val_accuracy):
        """
        Saves the model when validation-accuracy and/or accuracy has improved
        @param accuracy: Accuracy of the model in the last epoch
        @param val_accuracy: Accuracy of the model on the validation-subset
        """
        filename = (
            self.cfg.WEIGHTS_PATH
            + f"model_acc_{round(accuracy, 2)}"
            + f"_val_{round(val_accuracy, 2)}.pt"
        )
        if len(self.best) == 0:
            torch.save(self.model.state_dict(), filename)
            self.best.append((accuracy, val_accuracy, filename))
        elif val_accuracy > self.best[len(self.best) - 1][1] or (
            val_accuracy >= self.best[len(self.best) - 1][1]
            and accuracy > self.best[len(self.best) - 1][0]
        ):

            if len(self.best) == 1:
                delete = self.best[0][2]
                self.best.pop(0)
                os.remove(delete)
            torch.save(self.model.state_dict(), filename)
            self.best.append((accuracy, val_accuracy, filename))
            self.best = sorted(self.best, key=lambda t: (t[1], t[0]))
