# ReBike
ECE 499 Capstone Design Project - Regenerative Braking E-Bike
Main subsystems: Battery Pack, Motor Controls, User-Interface, CAN & Logging

## Battery Pack
The custom 36V, 8Ah battery pack with a JIKONG BMS monitors, protects, and balances individual cells, and provides SoC estimation over CAN. The lithium-ion cells are spot welded together with pure nickel strips in a 10 Series – 2 Parallel arrangement.
![BatteryPack](https://github.com/user-attachments/assets/63abeee7-6417-45e6-bf0a-9554f3fb5687)

## Motor Controls
DC power from the battery pack is converted to 3-ϕ AC power to drive the motor in the DSPIC33CK Motor Control Development Board. Field-Oriented Control (FOC) is implemented to isolate the torque-producing current and integrate regen braking. To mitigate safety risks, inverter temperature and current are monitored with fault detection. 
![image](https://github.com/user-attachments/assets/de39634b-ddfb-4037-8257-189e21daa129)

Nearly all geared e-bike motors are incapable of regen due to their freewheeling clutch. Hence, a direct drive PMSM is selected with a 45km/h top speed with a 36V battery.
![image](https://github.com/user-attachments/assets/eefcd2ba-b375-4a1b-8944-69aef89e761a)

## User-Interface
The Teensy 4.0 development board processes user inputs, delivers throttle commands over CAN, and displays the bike status on an OLED screen with custom graphics. 
![image](https://github.com/user-attachments/assets/fc436696-6336-41fa-87ed-38f2b3079e8a)
![image](https://github.com/user-attachments/assets/c99b8728-7308-40d0-a8d9-1a570724c5a5)

## CAN & Logging
Combining custom and Sol-Ark CAN protocols allows fast, noise-resilient inter-device communication on a single 500kbs bus. Throttle commands, screen values, and raw data are saved to an SD card for post-ride performance analysis. A dedicated CAN controller buffers incoming messages to the motor controller, protecting critical tasks from being interrupted.
![image](https://github.com/user-attachments/assets/1dda9d8c-1969-4304-8dcb-d35f7c6e3d53)

## Analysis
![image](https://github.com/user-attachments/assets/fa50669d-5dc0-42bf-a485-982481a41498)

![image](https://github.com/user-attachments/assets/3c0863f1-fffb-4b3b-bd07-b55b8a656b7d)

