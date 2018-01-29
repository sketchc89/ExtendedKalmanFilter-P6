# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

## Results

![Dataset 1](./Docs/dataset_1.gif)
![Dataset 2](./Docs/dataset_2.gif)

## Protocol

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

One change was made to CMakeLists.txt to support spdlog, a C++ logging library that eliminated the need for print statement debugging.

## Tests
The code was developed using the [Catch testing framework](https://github.com/catchorg/Catch2). 

* [Bash script](./src/tests/run_tests.sh) was written to run all of the tests. It completes the following steps.

* First compile tests_main.cpp in order to not have to recompile

```sh
$ g++ --std=c++11 tests_main.cpp -c
```

* Next compile tests

```sh
$ g++ --std=c++11 tests_main.o test_rmse.cpp -o test_rmse
$ ./test_rmse -r compact
```

* If changes are made to the test file, tests_main does not need to be recompiled
