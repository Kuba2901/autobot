# Autobot

Autobot is a ROS2-powered robot designed to resemble a car, equipped with Ackermann steering for smooth turns and precise control. It utilizes a Raspberry Pi 4 as the main controller, with an Arduino Uno driving the motor control and lighting. The robot is steered using a Gamepad for an intuitive control experience.

## Features
- **Ackermann Steering**: The robot uses Ackermann steering geometry, allowing for more realistic car-like movement and turning.
- **Gamepad Control**: The robot is controlled via a Gamepad, making it easy and fun to drive.
- **Raspberry Pi 4**: The brain of the robot, running ROS2 for all its control and communication.
- **Arduino Uno**: Handles motor control and lighting functions, interfacing with the Raspberry Pi 4.
- **Motor Driver Control**: Equipped with motor drivers to manage steering and movement.

## Technology Stack
- **ROS2**: The core framework for controlling and interacting with the robot.
- **Raspberry Pi 4**: Acts as the controller for the robot, running ROS2 nodes.
- **Arduino Uno**: Manages the physical steering and lighting hardware.
- **Gamepad**: Used for manual steering control.
- **Motor Drivers**: Control the robot's movement and lighting systems.

## Hardware Components
- Raspberry Pi 4
- Arduino Uno
- Motor drivers
- Gamepad for manual control
- Motors and steering system based on Ackermann geometry

## Getting Started
Clone this repository to your local machine and ensure that ROS2 is installed on your Raspberry Pi 4. Follow any setup instructions for connecting your Gamepad, motor drivers, and other hardware components.

