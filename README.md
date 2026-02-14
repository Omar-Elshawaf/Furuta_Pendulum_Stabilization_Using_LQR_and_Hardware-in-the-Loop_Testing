# Furuta_Pendulum_Stabilization_Using_LQR_and_Hardware-in-the-Loop_Testing

## Overview
This project focuses on the development of a **Furuta Pendulum** system, a popular example of an underactuated system used in control theory, especially in robotics and aerospace. The primary goal is to stabilize the pendulum in its upright position using a **Linear Quadratic Regulator (LQR)** controller. The system is modeled mathematically, and the controller is implemented both in **simulation** and in **real-world hardware** using **Hardware-in-the-Loop (HIL)** testing. The Furuta Pendulum system uses an **ESP32 microcontroller**, **DC motors**, **rotary encoders**, and a **state feedback controller**.

## Features
- **Furuta Pendulum System**: A rotary inverted pendulum system controlled to remain upright.
- **Mathematical Model**: Nonlinear dynamics derived using Euler–Lagrange methods, linearized for controller design.
- **Linear Quadratic Regulator (LQR)**: Controller designed to stabilize the pendulum using state feedback.
- **Hardware-in-the-Loop (HIL) Testing**: Real-time testing of the controller on physical hardware using Simulink models.
- **Real-time Data Visualization**: Python-based scripts for tuning LQR gains and real-time monitoring of control signals.
- **Parameter Estimation**: Experimental setup to estimate key parameters of the DC motor for accurate modeling.

## System Architecture
1. **Mechanical Design**: The system consists of a **rotary pendulum** mounted on a **rotary arm**, with sensors to track the angles of both the arm and the pendulum.
2. **Controller**: An **LQR controller** is used to stabilize the system, optimized using real-time adjustments.
3. **Electrical Design**: The motor control and sensor data are interfaced through an **ESP32 microcontroller**.
4. **Simulink Model**: A **Simscape** model is used for simulating the dynamics and controller performance.

## Key Components
- **DC Motor with Encoder**: Provides the actuation for the rotary arm and measures its angular velocity.
- **Rotary Encoder**: Measures the pendulum and arm angles.
- **ESP32**: Manages the control signals and communicates with the motor and sensors & H-Bridge motor control..
- **L298 H-Bridge**: Controls the direction of the DC motor.


## CAD
<img width="1920" height="866" alt="Futura_View_1" src="https://github.com/user-attachments/assets/8b020937-6618-4180-bc20-9694dbd9b4c8" />

<img width="1920" height="866" alt="Futura_View_2" src="https://github.com/user-attachments/assets/618e07f6-f9e6-47b8-93a1-b9422a680a74" />

<img width="1920" height="866" alt="Futura_View_3" src="https://github.com/user-attachments/assets/7c309cb6-6ec7-402a-9a77-4e37e50a8fa5" />

## Hardware
  <img width="525" height="327" alt="image" src="https://github.com/user-attachments/assets/aed1a756-dd07-4844-b544-a2670cf5cd0e" />


## Mathematical Model
The **Furuta Pendulum** dynamics are modeled using the **Euler–Lagrange method**, which results in a set of nonlinear differential equations. The model is then linearized around the upright equilibrium position to derive a **state-space** model suitable for controller design.

<img width="548" height="225" alt="image" src="https://github.com/user-attachments/assets/e9f609d1-9125-4f3d-ab11-bdd6ff001ef9" />

<img width="544" height="223" alt="image" src="https://github.com/user-attachments/assets/7ef1be1e-050f-43d6-916b-fa4f1d6f3c50" />

<img width="375" height="293" alt="image" src="https://github.com/user-attachments/assets/2578ca7e-e8fa-40a9-9ff5-65ee6c480fbb" />


The system is designed to handle **angular velocity**, **pendulum angle**, and the **motor voltage** through state-space matrices for optimal control.

## Controller Design
The **LQR controller** is used for state feedback to stabilize the system. The linearized state-space model forms the basis for the controller, and the gain matrices are optimized to minimize control effort while ensuring stability.

- **Gain Scheduling**: Multiple LQR gains are applied depending on the pendulum angle to improve stability across the entire motion range.
- **Python-based Tuning**: Fast tuning using Python scripts to generate new LQR gain matrices, accelerating the iteration process.

## Parameter Estimation
Parameter estimation is essential to accurately model the **DC motor** and ensure reliable control. Key motor parameters like **armature resistance**, **torque constant**, and **viscous friction coefficient** are estimated using experimental data and a Simulink model.

### Estimated Parameters:
- **Armature resistance (R)**: 4.857Ω
- **Torque constant (k_t)**: 0.9677 Nm/A
- **Viscous friction coefficient (b)**: 0.01398
- **Rotor inertia (J)**: 0.00702 kg·m²

## Hardware-in-the-Loop (HIL) Testing
The system was tested in a **Hardware-in-the-Loop (HIL)** setup where the real motor and pendulum system interact with the Simulink model. This step revealed practical challenges such as:
- **Sensor Noise**
- **Actuator Saturation**
- **Mechanical Inertia**

Key adjustments, such as **low-pass filtering** and **gain scheduling**, were introduced to stabilize the system under real-world conditions.

### Simulation and Real-Time Tuning:
- **Simulink Model**: Used to simulate the full system dynamics and controller.
- **Python Script**: Fast tuning of LQR gains through automated script generation, improving tuning efficiency.
- **Real-Time Data Visualization**: **DAQ scopes** and other tools were used to track system behavior in real-time.

## Conclusion
This project successfully developed a **Furuta Pendulum** system with **LQR control** and **Hardware-in-the-Loop** testing. The system demonstrated reliable performance in stabilizing the pendulum and was refined using real-time data and simulation. The approach bridges the gap between theoretical control design and real-world application, with future extensions planned for **nonlinear control** and **adaptive systems**.

## Output Video


https://github.com/user-attachments/assets/8b18da88-a5e2-4879-a3fa-dfbe7b425262



## Installation & Setup
### Hardware Setup:
1. Assemble the **pendulum system** with the **DC motor**, **rotary encoder**, and **ESP32**.
2. Connect the **ESP32** to the **motor**, **sensor**.

### Software Setup:
1. Set up the **Simulink model** in MATLAB to simulate the system.
2. Implement the **LQR controller** and fine-tune using the provided Python script for real-time adjustment of gains.
3. Run the **HIL testing** setup for real-time control.
4. For Real time LQR parameters Tuning , Use LQR.py and get the final Gain vlaue and put it in Simulink Bolck during Running
   <img width="1567" height="208" alt="image" src="https://github.com/user-attachments/assets/6686ba1a-d33e-4740-834b-794340e54ec7" />
<img width="1920" height="822" alt="image" src="https://github.com/user-attachments/assets/2c631645-a2f4-4f55-8834-3522b5bdfa1b" />




