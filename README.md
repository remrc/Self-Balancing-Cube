# Self-Balancing-Cube

**UPDATE (2024-07-14)**

New code is in **esp32_cube_enc** folder. I changed sensor orientation. So, if you made my cube before, you need to reprint one part to use this code.

Or you can print the redesigned cube https://www.thingiverse.com/thing:6695891

<img src="/pictures/cube2.jpg" alt="Cubli"/>

The blue connections you see in the schematic must be connected!

If something doesn't work, try the motors test sketch. It tests all motors, rotation directions and encoders.

Folow this video https://youtu.be/ZU0oTBRDgOE

---

ESP32, MPU6050, Nidec 24H brushless motors, 500 mAh LiPo battery.

Balancing controllers can be tuned remotely over bluetooth.

Example:

Send `p+` to increase `K1` by `1`, or send multiple at once, for example `p+p+p+p+p+p+` to increase `K1` by `6`. Sending `p-` decreases `K1` by the same amount. Below is the table of commands and how much they increase/decrease each parameter:

| Command | Parameter | Change |
| ------- | --------- | ------ |
| `p+`      | K1        | +1     |
| `p-`      | K1        | -1     |
| `i+`      | K2        | +0.05  |
| `i-`      | K2        | -0.05  |
| `s+`      | K3        | +0.005 |
| `s-`      | K3        | -0.005 |

<img src="/pictures/cube1.jpg" alt="Self-Balancing-Cube"/>

<img src="/pictures/schematic.png" alt="Self-Balancing-Cube-Schematic"/>

About the schematic:

* Battery: 3S1P LiPo (11.1V)
* Buzzer: any 5V active buzzer
* Voltage regulator: any 5V regulator (7805)
* All red connections are not necessary for this project! But if you are designing a PCB, I recommend making these connections. I might use encoders in the future, and you will be able to use the new firmware without any changes.
 
How to build:

https://youtu.be/AJQZFHJzwt4

If something doesn't work, try the [motors test sketch](https://github.com/remrc/Self-Balancing-Cube/blob/main/motors_test/motors_test.ino). It tests all motors, rotation directions and speeds. This helps you troubleshoot if the problem is in the software or hardware.

You can also make this balancing cube with an Arduino nano controller. All other parts remain the same.

<img src="/pictures/arduino_schematic.png" alt="Self-Balancing-Cube-Schematic"/>

In this version I made the offsets setting procedure simpler. First, connect to the controller over bluetooth. 
You will see a message that you need to calibrate the balancing points. Send `c+` from the serial monitor. This activates the calibration procedure. 
Set the cube to one of balancing points (edge or vertex). Hold still when the cube is perfectly balanced and send `c-` from serial monitor.
This will write the offsets to the `EEPROM`. Repeat this procedure four times (3 edges and vertex). After calibrating all offsets, the cube will automatically begin to balance.

ESP32 version also has an updated balancing point setting procedure. Important! In this video you can learn how to set the balancing points:

https://youtu.be/Nkm9PoihZOI


