# Development of a ROS Library in C++ for Dynamixel Motors

## Description

This Bachelor's Thesis project aims to develop a high-level library for Dynamixel motors, commonly used in various robotic applications. The library will be based on the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) (source development kit), which is Open Source and includes basic functions such as writing to and reading from the motor registers. This project is freely available to all members of the scientific community with the goal of facilitating motor control and improving efficiency.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Installation

To install this library, follow these steps:

1. Clone the repository:

    ```bash
    git clone https://github.com/TaISLab/dynamixel_ros_library.git
    cd dynamixel_ros_library
    ```

2. Install dependencies:

    ```bash
    sudo apt-get update
    sudo apt-get install ros-noetic-dynamixel-sdk
    ```

3. Build the library:

    ```bash
    catkin_make
    ```

4. Source the setup file:

    ```bash
    source devel/setup.bash
    ```

## Usage

To use the library in your project, include it in your CMakeLists.txt and package.xml files. Here's an example of how to integrate it:

### CMakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_ros_library
  CATKIN_DEPENDS roscpp dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

### package.xml

```xml
<depend>roscpp</depend>
<depend>dynamixel_sdk</depend>
```

After these changes, you can use the functionalities implemented in dynamixel_ros_library in your projects. Be sure to consult the examples located in the src folder, as they include the correct order of function calls for initializing control tables, creating object instances, and more.

## Features

The `dynamixel_ros_library` offers a robust and versatile set of features tailored for efficient and flexible control of Dynamixel motors within the ROS ecosystem. Below are the key features:

### Comprehensive Model Support

The library is compatible with all Dynamixel motor models, ensuring broad applicability across various robotic systems. While extensive testing has been conducted with the Dynamixel X-series, the library's design guarantees functionality with other models as well.

### Versatile Control Modes

Users can leverage any control mode offered by Dynamixel motors. This includes:

- **Position Control Mode**: Precise control over the motor's position.
- **Velocity Control Mode**: Accurate regulation of motor speed.
- **PWM Control Mode**: Direct control of the Pulse Width Modulation signal.
- **Extended Position Control Mode**: Allows for position control over an extended range.
- **Current Control Mode**: Enables direct control over the motor current.

### Simplified Register Access

Instead of directly manipulating registers, users select a parameter from the Dynamixel control table to modify. The library provides corresponding getter and setter methods for each parameter, streamlining the process of reading from and writing to motor registers.

### Error Handling and Validation

The library minimizes the possibility of errors, particularly in setter methods where users specify values. Each setter method includes a validation check to ensure the value is within the allowed range. If a value is out of range, an error message is displayed. This reduces the likelihood of user errors related to register values, as the library manages these assignments internally.

### ROS Integration

The `dynamixel_ros_library` is designed specifically for use with ROS (Robot Operating System). This integration allows for seamless communication and control within ROS-based robotic applications, enhancing the overall efficiency and effectiveness of motor control.

### Comprehensive Documentation and Examples

A detailed document is available, explaining each functionality and peculiarities of the library. Additionally, the `src` folder contains examples demonstrating the use of various methods, enabling users to quickly test and integrate the library with their motors.

### Extensibility

The library is designed to be easily modified by any user. This flexibility allows users to adapt the library to their specific needs or to extend its capabilities to suit particular requirements in their projects.

By leveraging these features, users can efficiently control and manage their Dynamixel motors within the ROS environment, facilitating a wide range of robotic applications.

## Contributing

## License

## Acknowledgements

Special thanks to the developers of the Dynamixel SDK and the ROS community for their unvaluable tools and support. This project would not have been possible without their contributions.

## Contact

For questions, feedback, or collaboration opportunities, please contact:

- Maksym Saldat
- [Contact by enail](mailto:maksymsaldat2001@gmail.com)
- [LinkedIn](https://www.linkedin.com/in/maksym-saldat/)