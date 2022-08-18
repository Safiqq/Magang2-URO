# Tubes Magang 2 Dagozilla

This project was built to read distances from 3 sensors (left, ahead, and right side) and run the motors. All of the sensors return value as microseconds, so we convert them to cm using formula: **time us * 0.0343 cm/us / 2**. Also the PWMs for the motors we use here is ranged from **[-1 .. 1]**, with -1 is spinning counter-clockwise and 1 is spinning clockwise. We convert the PWMs to duty cycle with 8 bits **[0 .. 255]** because *analogWrite* from Arduino.

## How to Use
1. Clone this repository
2. ```cd catkin_ws```
3. ```catkin_make```
4. ```source devel/setup.sh``` (terminal) or ```source devel/setup.zsh``` (bash)
5. Connect Arduino Mega
6. Open Arduino IDE, check on connected port, and upload **hardware.ino**
7. ```roslaunch command command.launch```
8. **INCASE ERROR caused by access:** ```sudo chmod a+rw /dev/ttyACM0```

## Notes:
- Port can be changed in **catkin_ws/src/command/launch/command.launch**
- If you are using Arduino Uno, change the *readSensorInt3()* procedure in **hardware.ino** to *ISR(PCINTx_vect)*. Reference: https://arduino.stackexchange.com/questions/70348/pcint0-pcint1-pcint2-etc-on-attiny45-85