# ROS2-based Color Block Sorting System using Universal Robot UR3e with TensorFlow Transfer Learning

This project is a ROS2 implementation of a color block sorting system using the Universal Robots UR3e and TensorFlow Transfer Learning. It aims to provide a comprehensive solution for automated sorting of colored blocks in a controlled environment using advanced robotics and machine learning techniques.

## Demonstration

https://github.com/Sushant369/ROS2-based-Color-Block-Sorting-with-Universal-Robot-UR3e-Using-TensorFlow-Transfer-Learning/assets/72655705/788e3021-c318-4fe9-bbeb-3fd7f07f4217

## Project Overview

The system uses a Universal Robot UR3e robotic arm to identify and sort blocks based on their color. TensorFlow Transfer Learning is employed to train a model capable of color recognition, which then guides the robotic arm to sort the blocks into designated areas. The implementation leverages ROS2 for managing the robot's operations and communications.

### Features

- **Color Detection**: Utilize TensorFlow to recognize different colored blocks.
- **Robotic Manipulation**: Control the UR3e robot to move and sort the blocks based on the detected color.
- **Parameterized Robot Movements**: Configure different movements for the robot through ROS2 parameters.
- **Real-Time Interaction**: Real-time control and feedback from the robot using ROS2 topics and services.

## System Requirements

- **ROS2 Humble**: Ensure ROS2 is installed and configured on your system.
- **Python 3.6+**: Required for running the scripts and the TensorFlow model.
- **Universal Robots UR3e Hardware**: This system is specifically designed for the UR3e model.

## Installation

Clone this repository to your local machine using:

```bash
git clone https://github.com/Sushant369/ROS2-based-Color-Block-Sorting-with-Universal-Robot-UR3e-Using-TensorFlow-Transfer-Learning.git
