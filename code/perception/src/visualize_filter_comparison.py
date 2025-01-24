#!/usr/bin/env python

import matplotlib.pyplot as plt
from math import pi

NEW_FILTER_FILE_NAME = "04"
OLD_FILTER_FILE_NAME = "04"
GT_FILE_NAME = "04"


# open the file with the estimated positions of the new filter
nf_pos_file = open(
    "/workspace/code/perception/src/experiments/filter_comparison/"
    + "new_filter_pos/data_"
    + NEW_FILTER_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
nf_pos_first_line = nf_pos_file.readline()

# open the file with the estimated headings of the new filter
nf_heading_file = open(
    "/workspace/code/perception/src/experiments/filter_comparison/"
    + "new_filter_heading/data_"
    + NEW_FILTER_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
nf_heading_first_line = nf_heading_file.readline()

# open the file with the estimated positions of the new filter
of_pos_file = open(
    "/workspace/code/perception/src/experiments/filter_comparison/"
    + "old_filter_pos/data_"
    + OLD_FILTER_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
of_pos_first_line = of_pos_file.readline()

# open the file with the estimated positions of the new filter
of_heading_file = open(
    "/workspace/code/perception/src/experiments/filter_comparison/"
    + "old_filter_heading/data_"
    + OLD_FILTER_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
of_heading_first_line = of_heading_file.readline()

# open the ground truth file
gt_file = open(
    "/workspace/code/perception/src/experiments/filter_comparison/"
    + "ground_truth/data_"
    + GT_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
gt_first_line = gt_file.readline()


# put all data into a 2-dimensional list of floats
nf_pos_lines = []
while True:
    next_line = nf_pos_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    nf_pos_lines.append(new_line)

# put all data into a 2-dimensional list of floats
nf_heading_lines = []
while True:
    next_line = nf_heading_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    nf_heading_lines.append(new_line)

# put all data into a 2-dimensional list of floats
of_pos_lines = []
while True:
    next_line = of_pos_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    of_pos_lines.append(new_line)

# put all data into a 2-dimensional list of floats
of_heading_lines = []
while True:
    next_line = of_heading_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    of_heading_lines.append(new_line)

# put all data into a 2-dimensional list of floats
gt_lines = []
while True:
    next_line = gt_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    gt_lines.append(new_line)


# times

nf_pos_time_stamps = []
for line in nf_pos_lines:
    nf_pos_time_stamps.append(line[0])

nf_heading_time_stamps = []
for line in nf_heading_lines:
    nf_heading_time_stamps.append(line[0])

of_pos_time_stamps = []
for line in of_pos_lines:
    of_pos_time_stamps.append(line[0])

of_heading_time_stamps = []
for line in of_heading_lines:
    of_heading_time_stamps.append(line[0])

gt_time_stamps = []
for line in gt_lines:
    gt_time_stamps.append(line[0])


# x positions

nf_x_positions = []
for line in nf_pos_lines:
    nf_x_positions.append(line[1])

of_x_positions = []
for line in of_pos_lines:
    of_x_positions.append(line[1])

gt_x_positions = []
for line in gt_lines:
    gt_x_positions.append(line[1])

# y positions

nf_y_positions = []
for line in nf_pos_lines:
    nf_y_positions.append(line[2])

of_y_positions = []
for line in of_pos_lines:
    of_y_positions.append(line[2])

gt_y_positions = []
for line in gt_lines:
    gt_y_positions.append(line[2])

# z positions

nf_z_positions = []
for line in nf_pos_lines:
    nf_z_positions.append(line[3])

of_z_positions = []
for line in of_pos_lines:
    of_z_positions.append(line[3])

gt_z_positions = []
for line in gt_lines:
    gt_z_positions.append(line[3])

# headings

nf_headings = []
for line in nf_heading_lines:
    nf_headings.append(line[1])

of_headings = []
for line in of_heading_lines:
    of_headings.append(line[1])

gt_headings = []
for line in gt_lines:
    heading_in_degrees = line[4]
    heading_in_rad = heading_in_degrees * pi / 180.0
    gt_headings.append(heading_in_rad)


def plot_x_position():
    plt.plot(
        nf_pos_time_stamps,
        nf_x_positions,
        label="nf x positions " + NEW_FILTER_FILE_NAME,
    )
    plt.plot(
        of_pos_time_stamps,
        of_x_positions,
        label="of x positions " + OLD_FILTER_FILE_NAME,
    )
    plt.plot(gt_time_stamps, gt_x_positions, label="gt x positions " + GT_FILE_NAME)
    plt.plot()
    plt.legend()
    plt.show()


def plot_y_position():
    plt.plot(
        nf_pos_time_stamps,
        nf_y_positions,
        label="nf y positions " + NEW_FILTER_FILE_NAME,
    )
    plt.plot(
        of_pos_time_stamps,
        of_y_positions,
        label="of y positions " + OLD_FILTER_FILE_NAME,
    )
    plt.plot(gt_time_stamps, gt_y_positions, label="gt y positions " + GT_FILE_NAME)
    plt.plot()
    plt.legend()
    plt.show()


def plot_z_position():
    plt.plot(
        nf_pos_time_stamps,
        nf_z_positions,
        label="nf z positions " + NEW_FILTER_FILE_NAME,
    )
    plt.plot(
        of_pos_time_stamps,
        of_z_positions,
        label="of z positions " + OLD_FILTER_FILE_NAME,
    )
    plt.plot(gt_time_stamps, gt_z_positions, label="gt z positions " + GT_FILE_NAME)
    plt.plot()
    plt.legend()
    plt.show()


def plot_heading():
    plt.plot(
        nf_heading_time_stamps, nf_headings, label="nf heading " + NEW_FILTER_FILE_NAME
    )
    plt.plot(
        of_heading_time_stamps, of_headings, label="of heading " + OLD_FILTER_FILE_NAME
    )
    plt.plot(gt_time_stamps, gt_headings, label="gt heading " + GT_FILE_NAME)
    plt.plot()
    plt.legend()
    plt.show()


plot_x_position()
plot_y_position()
plot_z_position()
plot_heading()
