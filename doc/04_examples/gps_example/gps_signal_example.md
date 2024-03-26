# GPS sensor

**Summary:** This page explains how the GPS sensor is handled including a short example on how to use it.

---

## Authors

Gabriel Schwald

### Date

07.01.2023

---
<!-- TOC -->
- [GPS sensor](#gps-sensor)
  - [Authors](#authors)
    - [Date](#date)
  - [Raw sensor data](#raw-sensor-data)
  - [Filters for the sensor data](#filters-for-the-sensor-data)
    - [Intuitive filter](#intuitive-filter)
    - [Rolling average](#rolling-average)
    - [Kalman Filter](#kalman-filter)
<!-- TOC -->

## Raw sensor data

The GPS sensor (GNSS) provides us with measurements for latitude, longitude and height.
While latitude and longitude are measured in degrees, altitude is measured in meters.

## Filters for the sensor data

As with all sensors provided by Carla, the GPS sensor output contains artificial noise.
![Unfiltered GPS signal](../../00_assets/filter_img/avg_1_w_1_000.png)
Right now there are multiple types of filters implemented.

### Intuitive filter

The first way combines the two most intuitive methods of smoothing the input.
At first, multiple ( $m$ ) inputs are summed up and divided to take an average.
Secondly, before updating the current position, the old position is saved.
The two positions are then added with a weight $w$, to adjust the responsiveness of the output signal.

The number of separate inputs taken into account for the average and the weight of old input can be seen as tweak-able
parameters.
The following graphs were taken while the car was stationary, the time on the bottom is therefore irrelevant.
Shown is the position translated to a local coordinate system, the transformation will be discussed later.

![GPS signal (m=1, w=0,5)](../../00_assets/filter_img/avg_1_w_0_500.png)
Using $w = 0.5$ clearly reduces the magnitude of the noise, however such a small value reduces the responsiveness
of the output signal.

![GPS signal (m=1, w=0,5)](../../00_assets/filter_img/avg_10_w_1_000.png)
Using a large number of data points ( $m = 10$ ) also improves the magnitude of the noise.
The main drawback here is the reduced frequency of the output signal, as the frequency of the output signal
is $\frac{1}{m}$ that of the input signal.
This can be avoided through the use of a rolling average where for every output
the last $m$ inputs are taken into account.

Combining these two parameters can lead to further improve the result.
![GPS signal (m=1, w=0,5)](../../00_assets/filter_img/avg_20_w_0_750.png)
The output signals frequency has now been reduced to 1Hz compared to the original 20Hz frequency,
with the weight now being set to $w = 0.75$

### Rolling average

Further improvements can be made by using a rolling average, where the last $m$ points are taken into account
for the average. The frequency does not change, as this average is calculated for every new point received.
This method is slightly more computationally intensive.

Instead of a single vector being kept from previous timesteps, a matrix with size $n x 3$ is computed
whenever a new signal is received.
Once new data is received the matrix is rotated by one position and the oldest measurement is overwritten.
The output is equal to the average of all $n$ vectors.

![Rolling average filter (n=20)](../../00_assets/filter_img/rolling_avg_20.png)

More arguments smooth out the gps signal, however the also add sluggishness to the output.
The number of arguments taken into account can be adjusted using the
[RUNNING_GPS_AVG_ARGS](../../../code/perception/src/position_heading_publisher_node.py) constant.

This was the method ultimately chosen with $n=10$, leading to the following gps signal.

![Final gps signal (n=10)](../../00_assets/filter_img/rolling_avg_10.png)

### Kalman Filter

A little more complex, but quicker reacting filter is the [Kalman Filter](../../06_perception/08_kalman_filter.md).

It is heavily dependent on which system model you use and how you tune its parameters.
When done correctly it reduces the GPS noise greatly without adding any delay to the output such as the filters above do.

![MAE Boxed Graph of Location Error with respect to ideal Location](../../../doc/00_assets/perception/data_26_MAE_Boxed.png)

In the upper graph a smaller box indicates less noise. Also the lower values are, the less deviation from the ideal position we have.

This is the graph that was used for tuning the kalman parameters:
![MSE Boxed Graph of Location Error with respect to ideal Location](../../../doc/00_assets/perception/data_26_MSE_Boxed.png)
It's depciting the MSE (mean squared errors) for the error distace to the ideal position.

As you can see the filtered Positions are still noisy, but way closer to the ideal position. In comparison, the running average filter is not as noisy, but constantly wrong by about 1 meter, because it is time delayed.

This Filter was tuned in simple situations (standing still, driving circles, driving farward), which results in worse performance when driving in more complex ways. (Complex Movements generally are not easy for the normal Kalman Filer to correct completely)

This is why the Kalman Filter performance can still be improved by using more complex models or a new more complex kalman filter, such as the non-linear or Extended Kalman Filter. Of course other filters may work better as well
