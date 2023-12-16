import torchvision
from PIL import Image
from torchvision.transforms import functional


def load_image(path):
    """
    Loads an image from the given path
    @rtype: RGB-coded PIL image
    """
    image = Image.open(path).convert('RGB')
    return image


class ResizeAndPadToSquare(object):
    def __init__(self, size):
        """
        Creates a transformation object that pads an image to have square size
            and resizes it to the given size
        @param size: Desired output size of the image
        """
        self.size = size

    def __call__(self, image):
        c, h, w = image.shape
        pad = abs(h - w) // 2
        if h < w:
            image = torchvision.transforms.Pad((0, pad))(image)
        else:
            image = torchvision.transforms.Pad((pad, 0))(image)
        image = torchvision.transforms.Resize(self.size)(image)
        return image


class Normalize(object):
    def __init__(self, mean, std):
        """
        Creates a transformation object that normalize an image by the
        given parameters
        @param mean: Sequence of means for each channel
        @param std: Sequence of standard deviations for each channel
        """
        self.mean = mean
        self.std = std

    def __call__(self, image):
        image = functional.normalize(image, mean=self.mean, std=self.std)
        return image


class ApplyMask(object):

    def __init__(self, path):
        """
        Creates a transformation object that masks an image with a grayscale
            mask
        @param path: Path to the mask
        """
        self.mask = functional.to_tensor(Image.open(path).convert('L'))

    def __call__(self, image):
        mask = torchvision.transforms.Resize(image.shape[1:])(self.mask)
        image = image * mask
        # plt.imshow(image.permute(1, 2, 0))
        # plt.show()
        return image
