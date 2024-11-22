# Filter Signal Package

This package contains tools to signal-process any ROS2 message.

## Table of Contents
1. [About this repository](#about-this-repository)
2. [Getting started](#getting-started)
3. [Building and Running](#building-and-running)


## About this repository

This package contains two ROS2 Nodes `analyse.py` and `filter.py` to analyse and then signal-process any ROS2 message. Currently
any float part of any message can be filtered with

- Highpass
- Lowpass
- Bandpass
- Bandstop

based on your configuration. 

### Analyse

Because any message type is supported, to filter a message you first have to analyse it. `analyse.py` will subscribe to a `topic` of your choice of arbitrary message type (as long as the message definition is sourced). It will then write the structure of the message as `result_file_name.yaml` into a specified `config_path`. The parameter:default_value are as following:

- `topic`:`imu`
- `config_path`:`~`
- `result_file_name`:`analytic_result` 

where `~` is the home-directory of the user executing the code. While the resulting file will be of type `.yaml`, do not include `.yaml` in the parameter, the file ending will be added automatically. E.g. result file for an `sensor_msgs.msg.Imu` looks like this:

```bash
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance:
- 0.0
- ...
- 0.0
header:
  frame_id: imu_frame
  stamp:
    nanosec: 413849829
    sec: 1732097342
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration_covariance:
- 0.0
- ...
- 0.0
orientation:
  w: 1.0
  x: 0.0
  y: 0.0
  z: 0.0
orientation_covariance:
- 0.0
- ...
- 0.0
```

### Filter

`filter.py` will subscribe to a topic of your choice and filter the topic according to the configuration file. The parameter:default_value are as following:

- `topic`:`imu`
- `config_path`:`~`
- `config_file_name`:`imu` 

where `~` is the home-directory of the user executing the code. While the configuration file will be of type `.yaml`, do not include `.yaml` in the parameter value, the file ending will be added automatically. Now, given the `analytic_result` from [Analyse](#analyse), we can edit it to become a `config_file` for the filter. For example, take the result file for an `sensor_msgs.msg.Imu`, and assume you want to filter 

- `angular_velocity.x` with a **lowpass** filter of **4**th order and a cuttoff-frequency of **20.0Hz**
- `angular_velocity.y` with a **highpass** filter of **2**nd order and a cuttoff-frequency of **1.0Hz**
- `linear_acceleration.x` with a **bandpass** filter of **3**rd order and a frequency band of **[5.0, 10.0]Hz**
- `linear_acceleration.z` with a **bandstop** filter of **2**nd order and a frequency band of **[3.0, 3.5]Hz**

then you would edit the file to look like:

```bash
angular_velocity:
  x: ['lowpass', 20.0, 4]
  y: ['highpass', 1.0, 2]
  z: 0.0
angular_velocity_covariance:
- 0.0
- ...
- 0.0
header:
  frame_id: imu_frame
  stamp:
    nanosec: 413849829
    sec: 1732097342
linear_acceleration:
  x: ['bandpass', [5.0, 10.0], 3]
  y: ['bandstop', [3.0, 3.5], 2]
  z: 0.0
linear_acceleration_covariance:
- 0.0
- ...
- 0.0
orientation:
  w: 1.0
  x: 0.0
  y: 0.0
  z: 0.0
orientation_covariance:
- 0.0
- ...
- 0.0
```

and load the edited file as the `filter` config file. Once the node is running it will publish into `topic/filtered`.

## Getting started

The current package looks like
```bash
filter_signal
    ├── CMakeLists.txt
    ├── config
    │   ├── analyse.yaml
    │   └── filter.yaml
    ├── filter_config
    │   ├── float32.yaml
    │   └── imu.yaml
    ├── filter_signal
    │   ├── __init__.py
    │   ├── analyse_parameters.py
    │   ├── analyse.py
    │   ├── filter_parameters.py
    │   ├── filter.py
    │   ├── filters.py
    │   └── utilities.py
    ├── include
    │   └── cpp_python_package
    ├── launch
    │   ├── analyse.launch.py
    │   └── filter.launch.py
    ├── package.xml
    ├── README.md
    └── src
```


## Building and Running
To build, please navitage into your workspace (adapt the first line in case it is different than `ros_ws`) and run

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
```

If you are using `colcon build --symlink-install` or e.g. using VSCode ROS2 Extention please make sure that all executable python files in
`ros_ws/src/fft/filter_signal/filter_signal` `filter.py` and `analyse.py` are made executable. Hence run

```bash
chmod +x filter.py
chmod +x analyse.py
```

Remember that only message types can be analysed and filtered that are also sourced. Hence if there is an interface package in your workspace (not installed in /opt/ros), make sure to also source the workspace before running. To run, proceed with [Analyse](#analyse) and [Filter](#filter). Note there exists `analyse.launch.py` and `filter.launch.py` launch files with associated `analyse.yaml` and `filter.yaml` configuration files for orientation.