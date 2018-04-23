# car1 control

Using sensor bumper to control steering and speed.

Manual start/stop by the back button.

Safe stop: automatic stop if the car senses no white track.

Remote control via VNC (without using static IP for Pi): 
+ turn off sensor bumper function.
+ Pi sends i2c commands to Arduino for start/stop, steering and speed control.
+ Pi reads i2c data back from Arduino.

Update client.py for wifi function: Pi reads from arduino and sends control commands to car2's Pi.



