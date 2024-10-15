#!/usr/bin/env python3
from argparse import ArgumentParser
from pathlib import Path
from shutil import rmtree, copyfile
import math
from imageio import v3 as iio
import numpy as np
import random


def create_argparse():
    argparser = ArgumentParser(description="CARLA Dataset Converter")
    argparser.add_argument("input_dir", help="Path to the input directory")
    argparser.add_argument("output_dir", help="Path to the output directory")
    argparser.add_argument(
        "--force",
        default=False,
        action="store_true",
        help="Overwrite output if already exists",
    )
    argparser.add_argument(
        "--shuffle",
        default=False,
        action="store_true",
        help="Shuffle the dataset before splitting it"
        " into train, test and validation sets",
    )
    return argparser


def convert_image_to_cityscapes_labelids(input_file: Path, output_file: Path):
    image = iio.imread(input_file)
    tag_ids = image[:, :, 0].astype(np.uint16) * 100000
    instance_ids = image[:, :, 1].astype(np.uint16) * 100
    instance_ids += image[:, :, 2].astype(np.uint16)
    converted_image = tag_ids + instance_ids
    iio.imwrite(output_file, converted_image)


def add_to_side_list(target_dict, side, file):
    if side not in target_dict:
        target_dict[side] = []
    target_dict[side].append(file)


def main():
    argparse = create_argparse()
    args = argparse.parse_args()
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)

    # define split sizes (could also be an argument)
    train = 0.8
    test = 0.1
    val = 0.1

    if output_dir.exists():
        if args.force:
            rmtree(output_dir)
            output_dir.mkdir()
        else:
            raise ValueError(
                f"given output_dir ({output_dir.as_posix()}) already exists!"
            )
    if not input_dir.is_dir():
        raise ValueError(f"input_dir ({input_dir.as_posix()}) needs to be a directory")

    # first create the necessary directories
    groundtruth = output_dir / "groundtruth"
    groundtruth.mkdir(parents=True)

    rgb_files = {}
    instance_files = {}

    # populate dicts
    for file in input_dir.rglob("*.png"):
        side = file.parts[-2]
        if "rgb" in file.parts:
            add_to_side_list(rgb_files, side, file)
        if "instance" in file.parts:
            add_to_side_list(instance_files, side, file)

    # sort images according to their sequence number
    for side in rgb_files:
        rgb_files[side] = sorted(rgb_files[side], key=lambda path: int(path.stem))
        instance_files[side] = sorted(
            instance_files[side], key=lambda path: int(path.stem)
        )

        print(f"rgb_files[{side}] length: {len(rgb_files[side])}")
        print(f"instance_files[{side}] length: {len(instance_files[side])}")

    splits = [train, test, val]
    split_names = ["train", "test", "val"]
    for side in rgb_files:
        print("processing", side)
        rgb_images = rgb_files[side]
        instance_images = instance_files[side]
        count_files = min(len(rgb_images), len(instance_images))

        if args.shuffle:
            # shuffle images
            shuffled = list(zip(rgb_images, instance_images))
            random.shuffle(shuffled)
            rgb_images, instance_images = zip(*shuffled)

        for split, name in zip(splits, split_names):
            rgb_target_dir = output_dir / side / name / "ds"
            groundtruth_target_dir = groundtruth / name / "ds"
            rgb_target_dir.mkdir(parents=True, exist_ok=True)
            groundtruth_target_dir.mkdir(parents=True, exist_ok=True)

            count_split = math.floor(count_files * split)
            rgb_split = rgb_images[:count_split]
            instance_split = instance_images[:count_split]
            rgb_images = rgb_images[count_split:]
            instance_images = instance_images[count_split:]

            for rgb_image, instance_image in zip(rgb_split, instance_split):
                image_id = rgb_image.stem
                rgb_file_name = f"ds_{side}_{image_id}.png"
                instance_file_name = f"ds_{side}_{image_id}_groundtruth.png"
                copyfile(rgb_image, rgb_target_dir / rgb_file_name)
                # convert_image_to_cityscapes_labelids(
                #    instance_image,
                #    groundtruth_target_dir / instance_file_name)   zzzyy
                copyfile(instance_image, groundtruth_target_dir / instance_file_name)


if __name__ == "__main__":
    main()
