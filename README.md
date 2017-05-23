# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

## Results

With the current setting, I get a RMSE below the rubric threshold (My RMSE on the test set is [0.080, 0.090,   0.333, 0.249]) This result is better than the results I had with the Extended KF (which is expected)

Here is the plot of my results (you can find an intearctive version [here](https://plot.ly/~2PetitsVerres/7/)

![Unscented KF results](/images/results.png)

### NIS value

The NIS value for both sensors is below the expected threshold (with respect with their degree of freedom) for more than 95% of the time. For the LIDAR, it is close to this mark of 95%, indicating that the tuning is probably close to a good value.

### Results using only one sensors

When using only one sensor, the RMSE increase (Radar only: [0.46, 0.53, 0.38, 0.44], Lidar only: [0.16, 0.15, 0.40, 0.24]), indicating that, as expected, the sensor fusion works better than one sensor only, both sensor giving additional information about the system state.

Here are the plots of the single sensors results (also available [here (radar)](https://plot.ly/~2PetitsVerres/9/) and [here (lidar)](https://plot.ly/~2PetitsVerres/11/)):

![Radar only](/images/radar.png)

![Lidar only](/images/lidar.png)


## Implementation detail

One important thing to handle in this project was the wrapping of the different angle. Angle in the state are wrapped between -pi and pi, as expected, but at some points, like when computing the mean predicted state, or the mean predicted measurement, there is a weighted average of angle that is computed. When doing so, if we have a weighted average, we must ensure that the angles used as input of the average are not wrapped.

For example, let's say that we have two angles to average with .5 weight for each, and the angles are close to pi. If they are for example [pi-eps, pi+eps], if they are normalized before the average, they become [pi-eps, -pi+eps] and the average becomes 0, where in fact it should be pi. To handle this problem, I have created a function UKF::NormalizeAngleArround(VectorXd A) that takes a vector of angles and normalize all of them between A0+pi and A0-pi, where A0 is the first vector element.

Apart from this detail, I think that my implementation is relatively straightforward.


---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.
