# Unscented Kalman Filter (based on SDC Nanodegree UKF project)

This project combines the Kalman and Extended Kalman filters with an unscented
Kalman filter for pedestrian/bicycle tracking.

The `FusionEKF` class is used to select which filter to use for each type of measurement.
Currently both RADAR and LIDAR use the Unscented Kalman Filter.  To switch back to
Kalman or Extended Kalman filters uncomment the appropriate calls.


## Basic Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
3. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt` or with another input

# Dependencies:

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

  # To run:

  `./build/UnscentedKF path/to/input.txt path/to/output.txt`

  e.g.

  `./build/UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt ./output.txt`
