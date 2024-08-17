# ReBike
ECE499 Capstone Design Project - Regenerative Braking E-Bike

Main subsystems: Battery Pack, Motor Controls, User-Interface, CAN & Logging

![image](https://github.com/user-attachments/assets/63a7b341-faa4-4de2-bde8-7ec958bbb726)

## Battery Pack
The custom 36V, 8Ah battery pack with a JIKONG BMS monitors, protects, and balances individual cells, and provides SoC estimation over CAN. The lithium-ion cells are spot welded together with pure nickel strips in a 10 Series – 2 Parallel arrangement.

![battery](https://github.com/user-attachments/assets/7516a325-2928-4e67-ae1d-192e10585e97)


![BatteryPack](https://github.com/user-attachments/assets/63abeee7-6417-45e6-bf0a-9554f3fb5687)

## Motor Controls
DC power from the battery pack is converted to 3-ϕ AC power to drive the motor in the DSPIC33CK Motor Control Development Board. Field-Oriented Control (FOC) is implemented to isolate the torque-producing current and integrate regen braking. To mitigate safety risks, inverter temperature and current are monitored with fault detection. Custom firmware was written on the Microchip dsPIC33CK Low Voltage Motor Control (LVMC) Development Board.

![FSM](https://github.com/user-attachments/assets/8ec09012-f001-452d-b119-f9a687071c2f)

![FOC](https://github.com/user-attachments/assets/e9fff6b6-0308-4521-be1c-9d897573b3ec)

![HallSector](https://github.com/user-attachments/assets/3d4a943b-a9f9-4bb3-86d2-6eddd20568f7)

![ElectricalAngle](https://github.com/user-attachments/assets/187961c2-eb00-414b-aa83-bf075bd27650)

Nearly all geared e-bike motors are incapable of regen due to their freewheeling clutch. Hence, a direct drive PMSM is selected with a 45km/h top speed with a 36V battery.

![image](https://github.com/user-attachments/assets/9190e69b-c699-4e04-985f-875e471d5432)

## User-Interface
The Teensy 4.0 development board processes user inputs, delivers throttle commands over CAN, and displays the bike status on an OLED screen with custom graphics.

![mountedUI](https://github.com/user-attachments/assets/f2bc4335-3e74-4e49-8721-dd39cdeaf837)

![Teensy](https://github.com/user-attachments/assets/ac6e376d-02cb-4840-ac5f-f827c2f4d538)

![flowchart](https://github.com/user-attachments/assets/854b580c-d32f-4331-8045-a99331eb3284)

![screengraphic](https://github.com/user-attachments/assets/a2c56918-8903-4689-9376-ca45ef3621f7)

## CAN & Logging
Combining custom and Sol-Ark CAN protocols allows fast, noise-resilient inter-device communication on a single 500kbs bus. Throttle commands, screen values, and raw data are saved to an SD card for post-ride performance analysis. A dedicated CAN controller buffers incoming messages to the motor controller, protecting critical tasks from being interrupted.

![CANdiagram](https://github.com/user-attachments/assets/1dda9d8c-1969-4304-8dcb-d35f7c6e3d53)

![CANformat](https://github.com/user-attachments/assets/e99ee30e-1115-4e31-b20e-5e668804d60c)

![Loggingtable](https://github.com/user-attachments/assets/5dc08cc5-000d-43fd-be2b-371af2df1aa1)

## Braking Strategy

![image](https://github.com/user-attachments/assets/fd2cd82d-07c1-4c3d-a58c-ca1307d723c7)

## Analysis

![systemperformance](https://github.com/user-attachments/assets/fa50669d-5dc0-42bf-a485-982481a41498)

![energyusage](https://github.com/user-attachments/assets/3c0863f1-fffb-4b3b-bd07-b55b8a656b7d)

## Assembly
![assembly](https://github.com/user-attachments/assets/c4344f35-8c44-48af-a716-08ad64e95df5)


