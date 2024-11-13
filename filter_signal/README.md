# ROS2 Package for C++ and Python Source Code

Template for a combined ROS 2 package which includes C++ and Python source code.

## Table of Contents
1. [About this repository](#about-this-repository)
2. [Getting started](#getting-started)
6. [Building and Running](#building-and-running)
3. [Steps to create a new combined package from scratch](#steps-to-create-a-new-combined-package-from-scratch)


## About this repository

This repository contains a variety of signal processing filters to apply to member of a ROS2 message that is 
of decimal type. 

## Getting started

The current repository looks like
```bash
    ├── CMakeLists.txt
    ├── config
    ├── filter_signal
    │   ├── __init__.py
    │   └── filter_signal.py
    ├── include
    │   └── cpp_python_package
    │       └── cpp_publisher.hpp
    ├── launch
    ├── package.xml
    ├── README.md
    └── src
        ├── cpp_publisher.cpp
        └── cpp_publisher_main.cpp
```


## Building and Running
To build, please navitage into your workspace (adapt the first line in case it is different than `ros_ws`) and run
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
```