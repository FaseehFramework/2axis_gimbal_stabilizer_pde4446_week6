# 2-axis Gimbal Stabilizer

A 2-axis mechanical gimbal that actively stabilizes a platform using a PID controller. The goal is to keep an object on the platform level, even when the gimbal's base is tilted or shaken.

## Authors
* Faseeh Mohammed (M01088120)
* William Kojumian (M01094013)

## Problem Statement
The core problem is to maintain the horizontal stability of a platform (gimbal surface) despite external physical disturbances, such as tilting or shaking, to prevent an object placed on it from falling.

* **Desired Variable:** The platform's angle of inclination (Pitch and Roll) relative to the horizontal plane must remain as close to 0° as possible.
* **Variables Under Control:** The angular position of the two SG90 servo motors (Pitch and Roll servos).
* **Expected Disturbances:**
    * External Tilting: Intentional or unintentional changes in the gimbal's base orientation.
    * Mechanical Shaking/Vibration: High-frequency, unpredictable movement leading to shakiness.
    * System Drift: Slow, consistent deviation of the platform from its level state due to minor sensor errors.
    * Initial Mechanical Misalignment: The physical servos were slightly tilted and not perfectly straight by default.

---

## Solution
Our solution is a 2-axis mechanical gimbal structure controlled by an embedded system.

* The MPU-6050 accelerometer/gyroscope measures the platform's current Pitch and Roll angles.
* These readings are fed into a dual-axis PID controller running on an Arduino Uno.
* The controller calculates the necessary motor adjustments to drive the angle error to zero.
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

## Gimbal Structure (3D Print)
The system relies on a custom printed frame. This structure includes a central holder, shaped like a hollow rectangular box, designed to protect and manage the internal wiring. At the top, the frame integrates mounts for the three SG90 servo extensions to ensure mechanical connection and proper articulation of the platform. 

![Mechanical Structure](https://raw.githubusercontent.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/869ed46f503f39d24770d184cb39c6af2cdb1aae/Pics/components%20stl.png) 

---
## Safety Mechanism
The safety design incorporates two key features: 

* Kill Switch (Emergency Stop): A physical switch is integrated to act as a kill switch. If any malfunction, dangerous movement, or instability occurs, flicking this switch immediately stops the system and cuts power to the actuators, preventing damage or runaway behavior. 
* PID Control Code: The core PID loop acts as a continuous software safety check, constantly monitoring the platform's angle and ensuring the stabilization is maintained.
---
## Circuit Diagram
* The system wiring is centered on the Arduino Uno. The MPU-6050 is connected via the  protocol ( and  pins). The three SG90 servo motors receive their control signals from digital PWM pins () but are powered by a separate, external  regulated supply to ensure sufficient current. All components share a common ground reference. The kill switch is wired to Digital Pin 2 using an input method (likely internal pull-up) to safely monitor the system's active state.
![Circuit Diagram](https://raw.githubusercontent.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/869ed46f503f39d24770d184cb39c6af2cdb1aae/Pics/circuit%20diagram.png) 
---
## Process
Our project development followed a systematic control systems approach, moving from conceptual design to mechanical realization and finally, iterative software optimization.

### 1. Conceptual Design & System Mapping
The project began with a clear definition of the system architecture and signal flow:
* **Input:** MPU-6050 (Pitch and Roll angles)
* **Controller:** Dual-axis PID loop (Arduino Uno)
* **Output:** PWM signals to SG90 Servos

![Control Loop Block Diagram](https://raw.githubusercontent.com/FaseehFramework/2axis_gimbal_stabilizer_pde4446_week6/869ed46f503f39d24770d184cb39c6af2cdb1aae/Pics/closed%20loop%20diagram.png) 

### 2. Mechanical Construction and Printing
The design was 3D printed and assembled.
* **Structure Assembly:** Assembling the hollow box holder, platform, and servo mounts.
* **Component Mounting:** Securing the three SG90 servo motors into their extensions and attaching the MPU-6050 to the platform.
* **Wiring Integration:** Connecting all components to the Arduino and breadboard.



### Initial Alignment and Software Compensation
During assembly, we encountered a significant mechanical flaw: the servo motors would not sit perfectly straight in the printed mounts, resulting in a slightly tilted default state. We compensated for this by introducing a code-based offset within the Arduino sketch to level the platform at rest.

```cpp
// Pitch (Servo 1, Pin 9)
int servo1Value = constrain(85 + pitchOutput, 0, 180);
servo1.write(servo1Value);

// Roll (Servo 2, Pin 8)
int servo2Value = constrain(79 - rollOutput, 0, 180);
servo2.write(servo2Value);
```
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
## Overcoming Data Overload
* We faced a major performance issue during testing: running the complex PID calculations while simultaneously printing diagnostic values (Pitch, Roll, P/I/D terms) to the Serial Monitor caused the Arduino Uno's microcontroller to overload and crash. 

* Solution: We used the serial output temporarily for critical debugging, but removed the continuous output callsfor the final production code. This significantly reduced the microcontrollers load, allowing the system to run flawlessly thereafter. 
---
## Result
* The final outcome successfully demonstrated the objective of creating a dynamically stable platform. After optimization with the manually tuned PID constants (), the closed-loop control system achieved high performance and stability. 

* The gimbal met expectations by performing the following: 

* Stability: The system reliably maintained the horizontal setpoint ( Pitch,  Roll) under various conditions of tilting and external disturbance. 

* Object Balancing: We tested the stability with several different objects, and in every case, the object remained stationary on the platform without falling, proving the effectiveness of the control logic. 

* Performance vs. Expectation: The final performance was highly satisfactory and very close to the desired expectation of achieving near-perfect level maintenance, successfully overriding the mechanical imperfections of the initial servo alignment.

[![YouTube Video](https://img.youtube.com/vi/uVE5MIrcOps/0.jpg)](https://www.youtube.com/watch?v=uVE5MIrcOps)

---

## Bonus: 3-Axis (Yaw) Implementation

The third servo is used for the Yaw axis, but it is not actively stabilized with a PID controller. This was a conscious design choice due to the limitations of the MPU-6050 sensor.

* **The Problem:** The MPU-6050 **lacks a magnetometer**. For Pitch and Roll, it can use the accelerometer (gravity) as an absolute "down" reference. For Yaw (rotation), it has no absolute reference and must rely *only* on the gyroscope.
* **Gyro Drift:** All gyroscopes suffer from **drift**. Tiny, unavoidable errors build up over time, causing the sensor's idea of "center" to constantly and slowly change.
* **Why PID Fails for Yaw:** A PID controller aggressively tries to "fix" this drift, seeing it as a real error. This causes the PID to constantly chase a moving target, resulting in the jitter and shakiness we observed in testing.
* **The Solution:** The final code uses a simple `map()` function for the yaw axis. This is a *relative* system. It doesn't fight the drift; it just follows the sensor's (drifting) idea of center. The whole system (sensor and servo) drifts together, which is far less noticeable than the jerky PID corrections.

<p align="center">
Here is a link to the demonstration video proving the stabilization capability of the system:<br>
<a href="https://youtu.be/uVE5MIrcOps">https://youtu.be/uVE5MIrcOps</a>
</p>

---
## Reflection
*Project Journey and Key Learnings*
This project offered significant hands-on experience in closing the loop between mechanical design and control software. Our journey reinforced several key engineering principles: 

The Primacy of PID Tuning: We gained a deep understanding of how the  (Proportional),  (Integral), and (Derivative) terms interact. We learned that  handles immediate responsiveness,  is essential for dampening the system's "shakiness," and  is critical for eliminating long-term system drift—a major win in achieving the final smoothness. 

*What Went Well*
* PID Tuning Success: The implementation of dynamic, serial-input PID tuning was highly successful, allowing us to quickly test and iterate on constants, which streamlined the optimization process. 

* Final Stability: The structure proved robust, and the final PID values resulted in a remarkably stable system that achieved the core objective of balancing objects reliably. 

* Collaborative Testing: The 50/50 partnership during the tuning phase, with William managing the code inputs and Faseeh providing physical feedback on stability, was highly efficient and effective. 

*What Went Wrong*
* Initial Mechanical Fit: The slight misalignment of the SG90 servos within the  print required unexpected software compensation, adding complexity to the setup phase. 

* Debugging Bottleneck: The system crashes caused by excessive data output to the Serial Monitor were frustrating but ultimately led to a better, leaner final code structure. We learned to rely on temporary serial outputs rather than continuous logging. 
---
## Contribution Matrix

| Task | William Kojumian | Faseeh Mohammed | Total
| :--- | :--- | :--- | :--- |
| **Project Concept & Design** | 50% | 50% | 100% |
| **Mechanical & 3D Structure** | 60% | 40% | 100% |
| **Electronics Assembly & Wiring** | 40% | 60% | 100% |
| **Software Development (Code)** | 40% | 60% | 100% |
| **Testing & PID Optimization** | 50% | 50% | 100% |
| **Documentation & Reporting** | 60% | 40% | 100% |
| **OVERALL PROJECT CONTRIBUTION** | 50% | 50% | 100% |

---


## References and Acknowledgements
* **3D-Printable Structure:** [https://thenoobinventor.github.io/arduino-gimbal/index.html](https://thenoobinventor.github.io/arduino-gimbal/index.html)
* **MPU-6050 Library (i2cdevlib):** [https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
* **PID Control Theory:** [https://playground.arduino.cc/Libraries/PIDLibrary/](https://playground.arduino.cc/Libraries/PIDLibrary/)
* > Gen AI Prompt/Input
> Based on the provided Arduino gimbal code that utilizes the #include "I2Cdev.h" and #include "MPU6050_6Axis_MotionApps20.h" libraries, and given access to the same hardware components (an MPU6050 sensor, two SG90 servos, and an Arduino Uno), how can a PID-tuned two-axis servo gimbal (controlling pitch and roll) be implemented?
---
