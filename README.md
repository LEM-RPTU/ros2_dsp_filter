# ROS2 Package for Signal Filtering utilizing FFT

ROS2 package to filter generic time series (e.g. IMU data) using DSP (Digital Signal Processing) tools.

![example](example.gif)
Example of a filtered signal consisting of three white noised sines with different amplitudes, frequencies and phase shifts.

## Table of Contents
1. [About this repository](#about-this-repository)
2. [Getting started](#getting-started)
6. [Building and Running](#building-and-running)
3. [Working with this Package](#working-with-this-package)


## About this repository
The repository consists of two standalone packages:
- `sample_signal`: This package allows you to create different signals consisting of various periodic elements enriched by white noise and/or a base signal to test the `filter_signal` package.
- `filter_signal`: This package applies an FFT transform to any specified ROS2 topic (which should be analyzed first) and displays the data in the frequency domain. Based on this analysis, the user can choose a suitable filter and apply it to the live topic to get a filtered result with a small delay.

Both repositories are documented with their own README files. It is advised to start with the `sample_signal` package if you are not very familiar with DSP.

We also highly recommend using [PlotJuggler](https://plotjuggler.io/) for data visualization purposes. Why wouldn't you!?
<p align="center"> 
<img src="plotjuggler_meme.png" alt="Example Image"> 
</p> 

## Getting started

The current repository tree looks like
```bash
ros2_dsp_filters
    ├── filter_signal
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── analyse.yaml
    │   │   ├── filter.yaml
    │   │   └── fourier.yaml
    │   ├── fft_result.png
    │   ├── filter_config
    │   │   ├── float32.yaml
    │   │   └── imu.yaml
    │   ├── filter_signal
    │   │   ├── analyse_parameters.py
    │   │   ├── analyse.py
    │   │   ├── filter_parameters.py
    │   │   ├── filter.py
    │   │   ├── filters.py
    │   │   ├── fourier_parameters.py
    │   │   ├── fourier.py
    │   │   ├── __init__.py
    │   │   └── utilities.py
    │   ├── fourier_config
    │   │   ├── float32.yaml
    │   │   └── imu.yaml
    │   ├── include
    │   │   └── cpp_python_package
    │   │       └── cpp_publisher.hpp
    │   ├── launch
    │   │   ├── analyse.launch.py
    │   │   ├── filter.launch.py
    │   │   └── fourier.launch.py
    │   ├── package.xml
    │   ├── README.md
    │   └── src
    │       ├── cpp_publisher.cpp
    │       └── cpp_publisher_main.cpp
    ├── README.md
    └── sample_signal
        ├── CMakeLists.txt
        ├── config
        │   ├── imu.yaml
        │   └── sample_signal.yaml
        ├── include
        │   └── cpp_python_package
        │       └── cpp_publisher.hpp
        ├── launch
        │   ├── imu.launch.py
        │   └── launch.py
        ├── LICENSE
        ├── package.xml
        ├── README.md
        ├── sample_signal
        │   ├── imu.py
        │   ├── __init__.py
        │   └── sample_signal.py
        ├── sample_signal.png
        ├── sine_wave.png
        └── src
            ├── cpp_publisher.cpp
            └── cpp_publisher_main.cpp
```
Where cpp part is not yet implemented and left as a placeholder. 

## Building and Running
This repository depends on `python3-scipy` and `python3-numpy`, hence please install
```
sudo apt-get install python3-scipy python3-numpy
```
or run `rosdep install --from-paths src -y --ignore-src` within your workspace. 
0. Create ros2 workspace and source folder, if needed:
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
``` 
1. Clone the package into the source of your ros2 workspace, source and build:
```bash 
git clone git@github.com:WICON-RPTU/ros2_dsp_filters.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
```
2. To check the default setup run in two different terminals:
```bash 
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sample_signal launch.py
```
and
```bash 
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch filter_signal filter.launch.py
```
You should see a result similar to the [example](example.gif) in the beginning of this Readme.

## Working with this Package
