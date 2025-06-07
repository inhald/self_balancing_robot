# Self-Balancing Robot

A real-time self-balancing two-wheeled robot powered by an STM32 microcontroller. The robot actively corrects its tilt using feedback from inertial sensors and a cascaded PID controller, restoring its upright posture within milliseconds of disturbance.

<p align="center">
  <img src="https://github.com/inhald/Self-Balancing-Robot/blob/main/chassis_photo.jpg" alt="Balancing Robot" width="400"/>
</p>

## Hardware Overview

- **Microcontroller:** STM32 DISCO Board  
- **Sensors:**  
  - On-board IMU (gyroscope + accelerometer)  
  - External ADXL345 accelerometer (for redundancy/testing)  
- **Actuators:**  
  - Two DC motors (differential drive)  
- **Motor Drivers:** H-Bridge motor driver (not shown in schematic)

## Control System

The robot uses a **cascaded PID controller** implemented in C++ with Mbed RTOS:

1. **Angle Estimation:**  
   Sensor fusion combines gyroscope and accelerometer data to compute pitch angle via complementary filtering.
2. **Inner Loop:**  
   A fast-acting PID controller adjusts motor torque to correct tilt angle in real time.
   
## Performance

- Stabilizes from ±15° tilt in < 100 ms  
- Maintains balance on flat and mildly uneven surfaces  
- Real-time operation under Mbed RTOS on bare-metal STM32
