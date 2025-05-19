# 🤖 SCARA Robotic Arm Drawing Project

This project is the final assignment for the *Robotic 1* course. It involves building a simple **SCARA robotic arm** capable of **drawing basic shapes such as circles and straight lines** using **inverse kinematics** and servo control via Arduino.

## 🎯 Objectives

- Design and build a physical SCARA robotic arm
- Implement forward and inverse kinematics
- Control servo motors using Arduino
- Simulate SCARA motion using Python
- Draw simple shapes on a 2D plane


## 🛠️ Hardware Used

- 2x MG996R Servo Motors (for joint rotation)
- 1x SG90 Servo Motor (for pen lift mechanism)
- Arduino Uno R3
- Acrylic or 3D printed SCARA frame
- Breadboard, jumper wires, 5V power supply

Detailed components list: [`hardware/component_list.md`](hardware/component_list.md)


## 💻 Software & Tools

- **Arduino IDE** – Programming the servo control logic
- **Python 3** – For simulating and testing inverse kinematics
- **Jupyter Notebook** – For interactive visualization
- **Fritzing** – (optional) For hardware diagram
- **PowerPoint / PDF** – For presentation and documentation


## 🚀 Features

- Input (x, y) coordinates to draw straight lines
- Automatically draw circles with defined radius and center
- Calculates joint angles using inverse kinematics
- Simulates SCARA arm movement before physical testing
- Pen up/down mechanism to switch between movement and drawing
