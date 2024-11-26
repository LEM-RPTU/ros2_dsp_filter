# ROS2 package for generating Sample Signal Messages

Contains code to generate sample signals to test both `fft` and `filter_signal` package capabilities.

## Table of Contents
1. [About this repository](#about-this-repository)
2. [Getting started](#getting-started)
3. [Building and Running](#building-and-running)


## About this package

There exists three python executables `imu.py`, `sine_wave.py` and `sample_signal.py` which generate a `sensor_msgs.msg.Imu`, a sine-wave in `std_msgs.msg.Float32` and a sample signal in `std_msgs.msg.Float32` respectively. The `launch.py` runs `sample_signal.py` with a configuration that allows you generate an arbitrary combination of 

- sine wave
- square wave
- triangle wave
- sawtooth wave
- pulse wave

and a base signal that can have linear of exponential growth. 

## Getting started

The current package looks like

```bash
sample_signal
    ├── CMakeLists.txt
    ├── config
    │   └── sample_signal.yaml
    ├── include
    ├── launch
    │   └── launch.py
    ├── LICENSE
    ├── package.xml
    ├── README.md
    ├── sample_signal
    │   ├── __init__.py
    │   ├── imu.py
    │   ├── sample_signal.py
    │   └── sine_wave.py
    └── src
```

Ignore `src` and `include`, they are only part of the used template to add CPP-files in the package later on.  


## Building and Running
To build, please navitage into your workspace (adapt the first line in case it is different than `ros_ws`) and run

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
```

Now, source your workspace and try
```bash
source install/setup.bash
ros2 run sample_signal imu.py
```
to get an empty `sensor_msgs.msg.Imu` publisher, run

```bash
source install/setup.bash
ros2 run sample_signal sine_wave.py
```

![Sine Wave](sine_wave.png "Sine Wave")

to get a sine wave of your chosen amplitude, frequency at your chosen topic and sample rate, and finally launch

```bash
source install/setup.bash
ros2 launch sample_signal launch.py
```

to generate a sample signal with e.g. an overlaying signals of types:

1. Sine at
  - 2Hz, 20 Amplitude, 0rad phase shift
  - 80Hz, 10 Amplitude, 1.57rad phase shift
  - 20Hz, 3 Amplitude, 3.14rad phase shift
2. Square wave
  - 0.5Hz, 25 Amplitude
3. Triangle wave
 - 30Hz, 24 Amplitude
4. Sawtooth Wave
 - 1.5Hz, 20 Amplitude
5. Pulse Wave 
  - 15Hz, 15 Amplitude, 0.5 Duty Cycle


at 200Hz publishing rate:

![Sample Signal](sample_signal.png "Sample Signal")
