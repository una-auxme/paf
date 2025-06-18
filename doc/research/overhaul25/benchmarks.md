# Benchmarks

**Summary:** This page contains benchmarks before/after the porting to ROS2

- [Hardware](#hardware)
- [Build time results](#build-time-results)
  - [ROS1 Build Run 1](#ros1-build-run-1)
- [Startup time results](#startup-time-results)
  - [ROS1 Startup Run 1](#ros1-startup-run-1)

[List of proposed benchmarks](./improvements/README.md#add-performance-benchmarks)

Only *build times* and *startup times* were measured.

## Hardware

The following hardware is used for the benchmarks:

Output from `sudo lshw -short`:

```csv
H/W path        Device          Class          Description
==========================================================
                                system         Alienware Aurora R13 (0AAB)
/0                              bus            0C92D0
/0/0                            memory         64KiB BIOS
/0/14                           memory         32GiB System Memory
/0/14/0                         memory         [empty]
/0/14/1                         memory         16GiB DIMM Synchronous 4800 MHz (0.2 ns)
/0/14/2                         memory         [empty]
/0/14/3                         memory         16GiB DIMM Synchronous 4800 MHz (0.2 ns)
/0/23                           memory         384KiB L1 cache
/0/24                           memory         256KiB L1 cache
/0/25                           memory         10MiB L2 cache
/0/26                           memory         25MiB L3 cache
/0/27                           memory         128KiB L1 cache
/0/28                           memory         256KiB L1 cache
/0/29                           memory         2MiB L2 cache
/0/2b                           processor      12th Gen Intel(R) Core(TM) i7-12700KF
/0/100                          bridge         12th Gen Core Processor Host Bridge/DRAM Registers
/0/100/1                        bridge         12th Gen Core Processor PCI Express x16 Controller #1
/0/100/1/0      /dev/fb0        display        GA102 [GeForce RTX 3080 Lite Hash Rate]
/0/100/1/0.1    card1           multimedia     GA102 High Definition Audio Controller
/0/100/1/0.1/0  input10         input          HDA NVidia HDMI/DP,pcm=8
/0/100/1/0.1/1  input11         input          HDA NVidia HDMI/DP,pcm=9
/0/100/1/0.1/2  input8          input          HDA NVidia HDMI/DP,pcm=3
/0/100/1/0.1/3  input9          input          HDA NVidia HDMI/DP,pcm=7
/0/100/4                        generic        Alder Lake Innovation Platform Framework Processor Participant
/0/100/6                        bridge         12th Gen Core Processor PCI Express x4 Controller #0
/0/100/6/0      /dev/nvme0      storage        PC801 NVMe SK hynix 2TB
/0/100/6/0/0    hwmon1          disk           NVMe disk
/0/100/6/0/2    /dev/ng0n1      disk           NVMe disk
/0/100/6/0/1    /dev/nvme0n1    disk           2048GB NVMe disk
/0/100/6/0/1/1  /dev/nvme0n1p1  volume         1074MiB Windows FAT volume
/0/100/6/0/1/2  /dev/nvme0n1p2  volume         1906GiB EXT4 volume
/0/100/8                        generic        12th Gen Core Processor Gaussian & Neural Accelerator
/0/100/14                       bus            Alder Lake-S PCH USB 3.2 Gen 2x2 XHCI Controller
/0/100/14/0     usb1            bus            xHCI Host Controller
/0/100/14/0/3                   input          AW-ELC
/0/100/14/0/6   input3          input          PixArt Dell MS116 USB Optical Mouse
/0/100/14/0/b                   input          Dell KB216 Wired Keyboard
/0/100/14/0/e                   communication  Bluetooth Radio
/0/100/14/1     usb2            bus            xHCI Host Controller
/0/100/14.2                     memory         RAM memory
/0/100/15                       bus            Alder Lake-S PCH Serial IO I2C Controller #0
/0/100/15.1                     bus            Alder Lake-S PCH Serial IO I2C Controller #1
/0/100/15.2                     bus            Alder Lake-S PCH Serial IO I2C Controller #2
/0/100/15.3                     bus            Alder Lake-S PCH Serial IO I2C Controller #3
/0/100/16                       communication  Alder Lake-S PCH HECI Controller #1
/0/100/17                       storage        Alder Lake-S PCH SATA Controller [AHCI Mode]
/0/100/19                       bus            Alder Lake-S PCH Serial IO I2C Controller #4
/0/100/19.1                     bus            Alder Lake-S PCH Serial IO I2C Controller #5
/0/100/1c                       bridge         Alder Lake-S PCH PCI Express Root Port #5
/0/100/1c/0     enp3s0          network        Killer E3000 2.5GbE Controller
/0/100/1c.7                     bridge         Alder Lake-S PCH PCI Express Root Port #8
/0/100/1c.7/0   wlp4s0          network        RTL8822CE 802.11ac PCIe Wireless Network Adapter
/0/100/1e                       communication  Alder Lake-S PCH Serial IO UART #0
/0/100/1f                       bridge         Z690 Chipset LPC/eSPI Controller
/0/100/1f/0                     system         PnP device PNP0c02
/0/100/1f/1                     system         PnP device PNP0c02
/0/100/1f/2                     system         PnP device PNP0c02
/0/100/1f/3                     system         PnP device PNP0c02
/0/100/1f/4                     system         PnP device PNP0c02
/0/100/1f.3     card0           multimedia     Alder Lake-S HD Audio Controller
/0/100/1f.3/0   input12         input          HDA Intel PCH Mic
/0/100/1f.3/1   input13         input          HDA Intel PCH Line
/0/100/1f.3/2   input14         input          HDA Intel PCH Line Out Front
/0/100/1f.3/3   input15         input          HDA Intel PCH Line Out Surround
/0/100/1f.3/4   input16         input          HDA Intel PCH Line Out CLFE
/0/100/1f.3/5   input17         input          HDA Intel PCH Line Out Side
/0/100/1f.3/6   input18         input          HDA Intel PCH Front Headphone
/0/100/1f.4                     bus            Alder Lake-S PCH SMBus Controller
/0/100/1f.5                     bus            Alder Lake-S PCH SPI Controller
/1                              power          To Be Filled By O.E.M.
/2              input0          input          Sleep Button
/3              input1          input          Power Button
/4              input2          input          Power Button
/5              input4          input          Dell KB216 Wired Keyboard
/6              input5          input          Dell KB216 Wired Keyboard System Control
/7              input6          input          Dell KB216 Wired Keyboard Consumer Control
/8              input7          input          Dell WMI hotkeys
```

## Build time results

3 runs of 3 builds of the agent are measured.

For ROS1, the command for building in the project root is:

```sh
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$(id -u -n)
docker build -f ./build/docker/agent/Dockerfile \
  --build-arg USERNAME=${USERNAME} \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  -t paf-agent-ros1 .
```

For each run:

- First build with the --no-cache option
- Second build after only changing the python requirements.txt (adding newline)
- Third build after only changing a python file (adding newline)

### ROS1 Build Run 1

1.

## Startup time results

3 runs of 3 startups each are measured with the [time_until_status](../../../../code/benchmark/time_until_status.py) node.

For ROS1 the [docker-compose.leaderboard.yaml](../../../build/docker-compose.leaderboard.yaml) is started with `docker compose up`.

For each run the system was freshly (re-)booted and three consecutive startups are executed:

- First with the container deleted/non-existing
- Second with the container deleted/non-existing
- Third with the existing container from the second startup

### ROS1 Startup Run 1

1.
