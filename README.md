# SELF BALANCING MOTORCYCLE
This project programing control a Self-balancing motorcycle using PID and LQG controller.
# Hardware 
  * C (Embedded programming),  AVR (Microcontroller)
  * MPU6050: Configured I2C for real-time data acquisition from the MPU6050
  * Brushless DC: PWM, Timer, Interrup for encoder
  * Module bluetooth HC05: UART communication
  * Servo MG996R
  * Pin Lipo 3S2P ...
# Software
  * Visual Studio Code: Programer for Microcontroller using PlatFormIO tool
  * MATLAB: Modeling, controller design, collect data from MPU6050 sensor
  * APP MIT INVENTOR: programming a control application for mobile devices
# Block Diagram
![image](https://github.com/user-attachments/assets/3efcf42c-b7a4-437b-a289-f3709f9eec02)
# Schematic Design:
![image](https://github.com/user-attachments/assets/79e8b8b9-b979-4bbc-903c-c377827e8496)
# Final Product:
![image](https://github.com/user-attachments/assets/70ece1a1-4ec8-44b7-9148-107723949859)
# App
![image](https://github.com/user-attachments/assets/a864c77f-6044-4a47-a436-e26dbbf1fb4a)
# Flowchar
![image](https://github.com/user-attachments/assets/b7b61bfe-80bc-43dc-8127-42bbd2bed26b)
#
First, establish a Bluetooth connection to the controller. A prompt will appear instructing you to calibrate the balance point. In the serial monitor, send the command c+ to initiate the calibration process. Position the bike at its balance point and keep it steady until it stops leaning to either side. Next, send the command c- through the serial monitor, which will store the offset values in the EEPROM. Once calibration is complete, the bike will start balancing on its own.
# VIDEO DEMO
https://drive.google.com/file/d/16y5qLCJKRR0sktt40f-jBoPtrOpci8vg/view?usp=sharing
