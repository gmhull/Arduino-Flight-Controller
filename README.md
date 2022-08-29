# Arduino-Flight-Controller
Custom flight controller using Arduino Uno.

This has been a long term project of mine.  It was heavily inspired by Joop Brokking's
auto-leveling quadcopter build.  I 3D printed the body of the drone to house the electronics.  The flight controller for the drone was built using an Arduino Uno.

![Drone Outside](./Drone.jpg?raw=true "Drone")

Parts for this project included:
- Arduino Uno
- MPU6050 Accelerometer/Gyro Chip
- BMP280 Barometer Chip
- 4x 2300kV Brushless Motors
- 4x 30a Electronic Speed Controllers
- 4x 5045 Propellers
- Power Distribution Board
- LED Light
- Wires/Resistors/Capacitor
- LiPo Battery, 3 cell 2200mAh

Here is a wiring diagram of the build:

![Wiring Schematic](./Wiring%20Schematic.png?raw=true "Wiring Schematic")

To set up the drone, first run the Drone_Setup file to save settings on the Arduino
with eeprom.  Drone_ESC_Calibrate can be used to test out the quadcopter before
flying.  Drone_Flight_Controller holds the flight controller code for the build.
