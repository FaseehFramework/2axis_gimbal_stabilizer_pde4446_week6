# 2-axis Gimbal Stabilizer

A 2-axis mechanical gimbal that actively stabilizes a platform using a PID controller. The goal is to keep an object on the platform level, even when the gimbal's base is tilted or shaken.

## Authors
* William Kojumian (M01094013)
* Faseeh Mohammed (M01088120)

## Demonstration

A video demonstrating the final stabilization capabilities and performance of the system.

[![Gimbal Demo](https://img.youtube.com/vi/uVE5MIrcOps/0.jpg)](https://youtu.be/uVE5MIrcOps)

---

## Project Objective

The core problem is to maintain the horizontal stability of a platform, ensuring its **Pitch** and **Roll** angles remain as close to 0° as possible. The system is designed to actively counteract external disturbances, including:
* External Tilting
* Mechanical Shaking and Vibration
* Sensor and System Drift
* Initial Mechanical Misalignment

---

## System Architecture and Control Loop

The solution is a closed-loop control system built on an Arduino Uno. The system follows this control flow:

1.  **Sensor:** An MPU-6050 accelerometer/gyroscope measures the platform's current Pitch and Roll angles.
2.  **Controller:** The Arduino Uno compares these angles to the desired setpoint (0° Pitch, 0° Roll) to generate an error signal.
3.  **PID Algorithm:** A dual-axis PID controller running on the Arduino calculates the necessary correction based on the error signal.
4.  **Actuator:** The controller sends PWM signals to the SG90 servo motors, which move the platform to counteract the error and maintain stability.

![Control Loop Block Diagram](https://raw.githubusercontent.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/869ed46f503f39d24770d184cb39c6af2cdb1aae/Pics/closed%20loop%20diagram.png) 

---

## Hardware Components

| Component | Purpose |
| :--- | :--- |
| **Arduino Uno** | Main microcontroller running the control loop and PID algorithm. |
| **MPU-6050** | Inertial Measurement Unit (IMU) used as the sensor to measure Pitch and Roll. |
| **3 x SG90 Servos** | Actuators for the Pitch, Roll, and Yaw axes. |
| **Kill Switch** | A physical switch for emergency stop control. |
| **Custom 3D-Printed Frame** | A custom-designed and printed frame to house all components and wiring. |
| **Buck Converter** | Steps down the 9V battery voltage to provide a stable, regulated power supply to the servos. |

---

## Circuit and Wiring

The system is centered on the Arduino Uno.
* **MPU-6050:** Connected via the I2C protocol (SDA and SCL pins).
* **Servos:** The control signals for the three servos are connected to the Arduino's digital PWM pins. The servos are **powered by an external 9V battery and buck converter** to ensure sufficient current, preventing the Arduino from browning out.
* **Kill Switch:** Wired to 9V source to act as an emergency stop.
* **Ground:** All components share a common ground reference.
  
![Circuit Diagram](https://raw.githubusercontent.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/869ed46f503f39d24770d184cb39c6af2cdb1aae/Pics/circuit%20diagram.png) 
---

## Safety Features

The project incorporates two key safety features:

1.  **Physical Kill Switch:** A switch is integrated into the circuit to act as an emergency stop. Flicking this switch immediately cuts power to the actuators, preventing damage or runaway behavior in case of a malfunction.
2.  **Software Safety Check:** The PID control loop itself acts as a continuous software safety check by constantly monitoring and ensuring the stabilization is maintained.

---

## Mechanical Design and Structure

The gimbal uses a custom 3D-printed frame. The main body is a hollow rectangular box designed to protect and manage the internal wiring. The frame includes integrated mounts for the three SG90 servo extensions to ensure a solid mechanical connection for the Pitch, Roll, and Yaw axes.

![Mechanical Structure](https://raw.githubusercontent.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/869ed46f503f39d24770d184cb39c6af2cdb1aae/Pics/components%20stl.png) 
---

## Challenges and Solutions

During development, we faced two major challenges:

1.  **Mechanical Misalignment:** The 3D-printed mounts did not hold the servo motors perfectly straight, resulting in a default tilt. This hardware flaw was corrected in software. We introduced a default angle offset in the code ( `85 + pitchOutput` and `79 - rolloutput`) to compensate for the physical misalignment, leveling the platform at rest.
   
![Code offset for Pitch and Roll]
(https://github.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/blob/main/Pics/code%20offset.png?raw=true) 

3.  **Microcontroller Overload:** In early tests, running the complex PID calculations while simultaneously printing diagnostic values to the Serial Monitor caused the Arduino Uno to overload and crash. This was solved by removing all continuous `Serial.print` calls from the final production code, which significantly reduced the microcontroller's load and made the system run flawlessly.

---

## PID Tuning

Optimizing the control loop was the most intensive part of the project. A dynamic tuning system was implemented that allows for adjusting PID constants in real-time by sending commands via the Serial Monitor. This avoided the need to re-upload code for every small change.

The tuning was sequenced by first tuning **$K_p$ (Proportional)** for responsiveness, then **$K_d$ (Derivative)** to dampen overshoot, and finally **$K_i$ (Integral)** to eliminate long-term drift.

### Final Optimized PID Constants:
* **$K_p$ (Proportional):** 1.2
* **$K_i$ (Integral):** 0.005
* **$K_d$ (Derivative):** 0.4
  
![PID Serial Input](https://github.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/blob/main/Pics/PIDtuning.png?raw=true) 
---

## ⭐ Bonus: 3-Axis (Yaw) Implementation

The third servo is used for the Yaw axis, but it is not actively stabilized with a PID controller. This was a conscious design choice due to the limitations of the MPU-6050 sensor.

* **The Problem:** The MPU-6050 **lacks a magnetometer**. For Pitch and Roll, it can use the accelerometer (gravity) as an absolute "down" reference. For Yaw (rotation), it has no absolute reference and must rely *only* on the gyroscope.
* **Gyro Drift:** All gyroscopes suffer from **drift**. Tiny, unavoidable errors build up over time, causing the sensor's idea of "center" to constantly and slowly change.
* **Why PID Fails for Yaw:** A PID controller aggressively tries to "fix" this drift, seeing it as a real error. This causes the PID to constantly chase a moving target, resulting in the jitter and shakiness we observed in testing.
* **The Solution:** The final code uses a simple `map()` function for the yaw axis. This is a *relative* system. It doesn't fight the drift; it just follows the sensor's (drifting) idea of center. The whole system (sensor and servo) drifts together, which is far less noticeable than the jerky PID corrections.

---

## 📚 References and Acknowledgements
* **3D-Printable Structure:** [https://thenoobinventor.github.io/arduino-gimbal/index.html](https://thenoobinventor.github.io/arduino-gimbal/index.html)
* **MPU-6050 Library (i2cdevlib):** [https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
* **PID Control Theory:** [https://playground.arduino.cc/Libraries/PIDLibrary/](https://playground.arduino.cc/Libraries/PIDLibrary/)

---

