# ESP32 Smart Line Following Robot with Traffic Sign Recognition

## Overview

This project is an autonomous line-following robot based on **ESP32**. The robot follows a black line using 5 infrared sensors and communicates with a computer through WiFi (TCP).

A camera system running on the computer detects traffic signs and sends commands to the ESP32. The robot can:

* Follow a line using PID control
* Detect and handle intersections
* Turn left or right based on traffic signs
* Stop at stop signs
* Avoid obstacles
* Communicate through WiFi TCP

---

## Hardware Components

### Main Controller

* ESP32 Development Board

### Sensors

* 5 IR Line Tracking Sensors

  * S1
  * S2
  * S3 (Center Sensor)
  * S4
  * S5

### Motor Driver

* L298N Motor Driver

### Motors

* 2 DC Gear Motors

### Communication

* WiFi TCP/IP

### External Vision System

* ESP32-CAM or USB Camera
* Computer running image recognition software

---

## Pin Configuration

### Line Sensors

| Sensor | GPIO   |
| ------ | ------ |
| S1     | GPIO32 |
| S2     | GPIO33 |
| S3     | GPIO25 |
| S4     | GPIO26 |
| S5     | GPIO27 |

### Motor Driver

| Signal | GPIO   |
| ------ | ------ |
| IN1    | GPIO19 |
| IN2    | GPIO21 |
| IN3    | GPIO17 |
| IN4    | GPIO18 |

---

## System Architecture

Camera → Image Processing → TCP Command → ESP32 → Robot Movement

### Workflow

1. Robot follows the line using PID.
2. Camera continuously detects traffic signs.
3. Computer sends commands through WiFi.
4. ESP32 receives commands.
5. At an intersection:

   * Left Sign → Turn Left
   * Right Sign → Turn Right
   * Stop Sign → Stop for 10 seconds
   * Obstacle Sign → Execute avoidance maneuver
6. Robot continues following the line.

---

## PID Line Following

### Error Calculation

Sensor weights:

| Sensor | Weight |
| ------ | ------ |
| S1     | -2     |
| S2     | -1     |
| S3     | 0      |
| S4     | 1      |
| S5     | 2      |

Formula:

```text
Error = Sum(weights of active sensors) / Number of active sensors
```

Example:

```text
00100 → Error = 0
01100 → Error = -0.5
00011 → Error = 1.5
```

### PID Formula

```text
Motor Output = Kp × Error + Kd × Derivative
```

Current parameters:

```cpp
Kp = 33
Kd = 305
BaseSpeed = 70
```

---

## Intersection Detection

An intersection is detected when:

```cpp
activeSensors >= 3
AND
(S1 == 0 OR S5 == 0)
```

Examples:

```text
00100 → Normal Line
01110 → Possible Intersection
11111 → No Line
```

---

## Supported Commands

### Left Turn

```text
leftSign
```

Robot turns left at the next intersection.

### Right Turn

```text
rightSign
```

Robot turns right at the next intersection.

### Stop Sign

```text
stopSign
```

Robot:

* Aligns to the line
* Stops completely
* Waits 10 seconds
* Continues operation

### Obstacle

```text
obstacle
```

Robot performs obstacle avoidance and returns to the line.

---

## WiFi Configuration

```cpp
const char* ssid = "Hositan";
const char* password = "12345679";
```

TCP Port:

```cpp
6868
```

---

## Project Structure

```text
ESP32-LineFollower/
│
├── Robot.ino
├── README.md

---

## Main Features

✅ PID Line Following

✅ WiFi TCP Communication

✅ Traffic Sign Recognition Integration

✅ Autonomous Intersection Handling

✅ Obstacle Avoidance

✅ Stop Sign Handling

✅ Real-Time Sensor Monitoring

---

## Future Improvements

* Full PID (Add Integral Term)
* Dynamic Speed Adjustment
* ESP32-CAM Onboard Recognition
* Path Planning Algorithm
* SLAM Navigation
* Mobile Application Monitoring

---

## Author

Ho Si Tan

Foreign Trade University

IoT & Embedded Systems Project

2026
