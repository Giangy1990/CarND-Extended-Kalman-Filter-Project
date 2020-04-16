# Extended Kalman Filter
---
## Purpose
This project implements an Extended Kalman Filter (EKF) to estimate the state of a moving object detected using Laser and Radar measurements. To evaluate the goodness of the EKF, the Root Mean Squared Error (RMSE) between the estimation provided by the EKF algorithm and a provided ground truth has been computed.

## Dependencies
* Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).
* uWebSocketIO
  * Linux and Mac: This repository includes two files that can be used to set up and install * [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
  * Windows: you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Simulation protocol
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.\
**INPUT**: values provided by the simulator to the c++ program

    ["sensor_measurement"] => the measurement that the simulator observed (either laser or radar)
**OUTPUT**: values provided by the c++ program to the simulator

    ["estimate_x"] <= kalman filter estimated position x

    ["estimate_y"] <= kalman filter estimated position y

    ["rmse_x"]

    ["rmse_y"]

    ["rmse_vx"]

    ["rmse_vy"]

[//]: # (Image References)

[image1]: fig/dataset1.PNG "Dataset1"
[image2]: fig/dataset2.PNG "Dataset2"
    ---
## Algorithm
The program uses the first measurement to initialize all the matrices of the kalman filter. From the second measurement on, it performs the following steps:
1. compute the elapsed time from the previous measurement
2. predict the object state
3. update the filter parameters using:
  * EKF equations if the measurement comes from the Radar
  * KF equations if the measurement comes from the Laser

This algorithm can be called using the method ***ProcessMeasurement(...)*** of the class **FusionEKF** defined in the file [FusionEKF.h](src/FusionEKF.h).
For what concern the ***Predict()***, ***Update(...)*** and ***UpdateEKF(...)*** methods, they are defined in the class **KalmanFilter** which implementation can be found in the file [kalman_filter.cpp](src/kalman_filter.cpp).

### Initialization details
The initialization of the **FusionEKF** class is split between the class constructor ( line 17-63 of the file [FusionEKF.cpp](src/FusionEKF.cpp) ) and the begin of the ***ProcessMeasurement(...)*** method ( line 76-111 of the file [FusionEKF.cpp](src/FusionEKF.cpp) ).\
In the class constructor, there are the initializations of the elements that don't depend from the measurements:
* `R_laser_` and `R_radar_` measurement covariance matrices (one for Laser and one for Radar);
* `H_laser_` laser measurement matrix (the Radar one is not static and so it is computed at runtime);
* `P_` state covariance matrix;
* `F_` transition matrix.

On the other hand, in the ***ProcessMeasurement(...)*** method, the first measure is used to initialize:
* `previous_timestamp_` time memory variable;
* `x_` state vector.

For the state vector `x_` there are two procedures according to the nature of the input measurement:
* Laser: direct initialization cooping the measured position into the state;
* Radar: conversion from polar to cartesian coordinates and fill the state.

### Predict details
To perform the prediction step, it is necessary to perform three computations before the ***Update(...)*** method call:
* compute the elapsed time from the previous measurement;
* update the transition matrix `F_` according to the elapsed time;
* update the process noise covariance matrix `Q_`.

During the coding of the last operation, the number of computation have been reduced allocating temporary variables to store operations' results that will be reused. (according to the point in the rubric "avoid unnecessary computations")

### Update details
In order to handle both Radar and Laser measurements, after the predict step there is an `if` statement that discern the procedure to perform.\
In case of **Radar** measurement
* the measurement matrix is filled using the ***CalculateJacobian(...)*** method of the **Tools** class;
* the measurement covariance matrix is filled with `R_radar_`;
* the ***UpdateEKF(...)*** method is called.

In case of **Radar** measurement
* the measurement matrix is filled with `H_laser_`;
* the measurement covariance matrix is filled with `R_laser_`;
* the ***Update(...)*** method is called.

The difference between ***UpdateEKF(...)*** and ***Update(...)*** is in the measurement function.
***Update(...)*** uses the linear equation `z_pred = H_*x_`, while ***UpdateEKF(...)*** uses the non linear equation `z_pred = h(x_)` where `h(x_)` performs the cartesian to polar transformation.

## Results
The metric used to evaluate the accuracy of the EFK, is the RMSE. Its computation is performed by the function method ***CalculateRMSE(...)*** of the Tools class ( implementation in line 16-42 of [tools.cpp](src/tools.cpp) file ).\
Evaluation has been conducted on both Dataset1 and Dataset2 avaiable in the simulator.\
The following table resumes the results and the baseline provided in the [rubric](https://review.udacity.com/#!/rubrics/748/view).

|    Parameter    |    Baseline    |    Dataset1    |    Dataset2    |
|:---------------:|:--------------:|:--------------:|:--------------:|
|    RMSE x       |    0.11        |    0.0973      |    0.0726      |
|    RMSE y       |    0.11        |    0.0855      |    0.0967      |
|    RMSE vx      |    0.52        |    0.4513      |    0.4579      |
|    RMSE vy      |    0.52        |    0.4399      |    0.4966      |

In the following figure it is possible to see the EKF results for both datasets.
* **Dataset1** with both sensors

![alt text][image1]

* **Dataset2** with both sensors

![alt text][image2]

To figure out the performances of the single sensor, other two test were performed using one sensor at time.\
In both cases, the resulting RMSE was bigger than the required one but, as expected, the position estimated by the Laser was more accurate of the one estimated using the Radar.
