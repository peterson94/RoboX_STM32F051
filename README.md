2WD Robot Car with implemented speed regulation and remote control.
- Speed measurement with optical encoders, traced back to pulse-width time measurement
- Speed regulation with implemented PI controller
- Direction command messages sent via Bluetooth/UART interface

<p float="left">
<img src="https://github.com/peterson94/RoboX_STM32F051/blob/main/Gallery/20230823_154839.jpg" alt="Pic1" width="240" height="320">
<img src="https://github.com/peterson94/RoboX_STM32F051/blob/main/Gallery/20230823_154855.jpg" alt="Pic2" width="240" height="320">
<img src="https://github.com/peterson94/RoboX_STM32F051/blob/main/Gallery/20230823_154907.jpg" alt="Pic3" width="240" height="320">
</p>

Usage Manual:
1) Dowload STM32CubeMX Version 6.9.0 and import .ioc file to make C project with "skeleton"
2) Download STM32CubeIDE Version 1.13.0 and open generated C project
3) Copy Core/* content into the correspondinhg path in the project
4) Edit if needed, then build and run

Items used:
- TT DC gear motors
- Microcontroller: STM32F051K8T6
- Motor Driver: L298N module
- Optical encoders: LM393 modules
- Battery (2 pcs CR18650)
- MP2315 DC/DC converter module
- Homemade boards for battery measurement and power supply distribution
- Plexiglass chassis on bottom (transparent)
- PLA 3D printed chassis on top (blue)
- HC-05 bluetooth module

Softwares used:
- Program skeleton: STM32 CubeMX
- Program coding: STM32 CubeIDE (embedded C project)
- Control panel: Bluetooth Controlled Joystick for Android (Uncia Robotics)
