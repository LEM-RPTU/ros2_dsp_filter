# ROS2 Package for C++ and Python Source Code

Template for a combined ROS 2 package which includes C++ and Python source code.

## Table of Contents
1. [About this repository](#about-this-repository)
2. [Getting started](#getting-started)
6. [Building and Running](#building-and-running)
3. [Steps to create a new combined package from scratch](#steps-to-create-a-new-combined-package-from-scratch)


## About this repository

This repository contains an exemplary ROS2 package which contains C++ and Python source code. More precisely, it is a C++ package which is extended by Python code and the necessary modifications when building. This package demonstrates this by including a publisher which is written in C++ and a subscriber which is written in Python. Good additional manuals to this can be found here:
- [Create a ROS2 package for Both Python and Cpp Nodes](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)
- [ament_cmake_python user documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)

## Getting started

The current repository looks like
```bash
    ├── CMakeLists.txt
    ├── config
    ├── cpp_python_package
    │   ├── __init__.py
    │   └── py_subscriber.py
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
where all C++ source code is in `src` (with its header files in `include/cpp_python_package`) and all Python source code in the subfolder `cpp_python_package`. Note that the file `__init__.py` in the folder `cpp_python_package` is necessary and that all Python source files must have the line 
```python
#!/usr/bin/env python3
```
on top of the file (see `cpp_python_package/py_subscriber.py`).

## Building and Running
To build, please navitage into your workspace (adapt the first line in case it is different than `ros_ws`) and run
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
```
Now, source your workspace and run the C++ publisher
```bash
source install/setup.bash
ros2 run cpp_python_package cpp_publisher
```
and open another terminal which runs the Python subscriber
```bash
source install/setup.bash
ros2 run cpp_python_package py_subscriber.py
```
You should see that the Python subscriber receives the values published from the C++ publisher.

## Steps to create a new combined package from scratch
-   Create a C++ package:
    ```bash
    ros2 pkg create --build-type ament_cmake <package_name>
    ```

-   In the new package folder, create a new folder with the same name as the package. This is the folder for the Python source code files. Inside this folder, it is necessary to create an empty file `__init__.py` which shows Python that this folder is a Python module.

-   Edit the file `CMakeLists.txt` of the new package by adding the following lines:
    -   Below the line `find_package(ament_cmake REQUIRED)`, add the line
        ```cmake
        find_package(ament_cmake_python REQUIRED)
        ```
        and any other dependencies of your project, e.g.
        ```cmake
        find_package(rclcpp REQUIRED)
        find_package(rclpy REQUIRED)

        find_package(geometry_msgs REQUIRED)
        ```
    -   After the sections where the Cpp dependencies and executables are defined, add the following part for the Python files:
        ```cmake
        # Install Python modules
        ament_python_install_package(${PROJECT_NAME})

        # Install Python executables
        install(PROGRAMS
            package_name/python_script.py
            DESTINATION lib/${PROJECT_NAME}
        )
        ```
        where you have to adapt `package_name/python_script.py` to your package name and the file name of your Python script (ensure that the line `#!/usr/bin/env python3` is included on top of your Python file).
    
-   Edit the file `package.xml` by adding the following lines:
    -   Below the line `<buildtool_depend>ament_cmake</buildtool_depend>`, add the line
        ```xml
        <buildtool_depend>ament_cmake_python</buildtool_depend>
        ```
        and any other dependencies of your project, e.g.
        ```xml
        <depend>rclcpp</depend>
        <depend>rclpy</depend>
        <depend>geometry_msgs</depend>
        ```

