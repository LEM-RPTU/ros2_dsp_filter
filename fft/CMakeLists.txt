cmake_minimum_required(VERSION 3.8)
project(fft)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


# Include "include" directory for header files
# include_directories(include)

# Create cpp executable
# add_executable: <name> <source file to executable> <List other required source files>
# add_executable(cpp_publisher src/cpp_publisher_main.cpp
#                src/cpp_publisher.cpp)


# Target dependencies
# set(cpp_publisher_dependencies rclcpp geometry_msgs)
# ament_target_dependencies(cpp_publisher ${cpp_publisher_dependencies})

# Install required directories (e.g. launch, config)
install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
# install(TARGETS
#   cpp_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  fft/plotfourier.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
