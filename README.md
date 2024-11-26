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


## Working with this Package
