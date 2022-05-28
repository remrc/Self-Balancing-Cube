# Self-Balancing-Cube

ESP32, MPU6050, Nidec 24H brushless motors, 500 mAh LiPo battery.

Balancing controllers can be tuned remotely over bluetooth.

Example:

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

The same for K2, K3. Send "i", "s".

<img src="/pictures/cube1.jpg" alt="Self-Balancing-Cube"/>

<img src="/pictures/schematic.png" alt="Self-Balancing-Cube-Schematic"/>

About schematic:

Battery: 3S1P LiPo (11.1V). 
Buzzer: any 5V active buzzer.
Voltage regulator: any 5V regulator (7805).
All red connections not nescesary for this project! But if you are designing a PCB I recommend making these connections. Maybe I use encoders in the future, you will be able to use the new firmware without any changes.
 
How to build:

https://youtu.be/AJQZFHJzwt4

