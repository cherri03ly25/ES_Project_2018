# Source codes developed for ES project - group G11-CPR: Basic Platform for Vehicle Platooning

Updated servo control, i2c and wifi communication:
- steering and speed control (Servo.h)
- i2c communication between Arduino (Wire.h) and RPi (smbus2)
- wifi socket communication between two RPis (socket)
- remote control via VNC/SSH

Implementation: Server car just imitates client car
- Client Pi reads running status and control parameters from arduino and sends data to server Pi.
- Server Pi reads running status and control parameters from client Pi and sends control data to arduino.






