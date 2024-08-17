# ReBike
ECE 499 Capstone Design Project - Regenerative Braking E-Bike
Main subsystems: Battery Pack, Motor Controls, User-Interface, CAN & Logging
![Bike](https://github.com/user-attachments/assets/90abbd77-cab0-44f5-af45-63a184b6bf8d)

## Battery Pack
The custom 36V, 8Ah battery pack with a JIKONG BMS monitors, protects, and balances individual cells, and provides SoC estimation over CAN. The lithium-ion cells are spot welded together with pure nickel strips in a 10 Series – 2 Parallel arrangement.
![BatteryPack](https://github.com/user-attachments/assets/63abeee7-6417-45e6-bf0a-9554f3fb5687)

## Motor Controls
DC power from the battery pack is converted to 3-ϕ AC power to drive the motor in the DSPIC33CK Motor Control Development Board. Field-Oriented Control (FOC) is implemented to isolate the torque-producing current and integrate regen braking. To mitigate safety risks, inverter temperature and current are monitored with fault detection. 
![image](https://github.com/user-attachments/assets/51c1b568-a490-4832-821c-a24938c7d21f)

Nearly all geared e-bike motors are incapable of regen due to their freewheeling clutch. Hence, a direct drive PMSM is selected with a 45km/h top speed with a 36V battery.
![image](https://github.com/user-attachments/assets/27c9228c-bbc9-4513-9e4e-67bf466d2379)

## User-Interface
The Teensy 4.0 development board processes user inputs, delivers throttle commands over CAN, and displays the bike status on an OLED screen with custom graphics. 
![image](https://github.com/user-attachments/assets/3597d601-2d33-4ad5-bb00-f18dfe6e9bae)
![image](https://github.com/user-attachments/assets/3570ea68-9117-4878-9c52-6d728153720e)

## CAN & Logging
Combining custom and Sol-Ark CAN protocols allows fast, noise-resilient inter-device communication on a single 500kbs bus. Throttle commands, screen values, and raw data are saved to an SD card for post-ride performance analysis. A dedicated CAN controller buffers incoming messages to the motor controller, protecting critical tasks from being interrupted.
![image](https://github.com/user-attachments/assets/1dda9d8c-1969-4304-8dcb-d35f7c6e3d53)

## Analysis
![image](https://github.com/user-attachments/assets/2af54e03-ec7b-4a21-a090-1acd016669b5)

![image](https://github.com/user-attachments/assets/8aa1d218-3aee-4627-97cf-61002b5a8c7d)
