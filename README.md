Quaduino
====================

This is the code of my homebrew quadcopter, and RemoteController.
Both use Arduino Uno Board as controller.

List of modules I am using:

  nRf24L01+
  
  MPU6500
  
List of Library I am using:

  RF24 -- https://github.com/maniacbug/RF24
  
  i2cdevlib -- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
  

I set my quad in + mode and use PID controller for Pitch,Roll angular rate control.
And a PD controller for Pitch,Roll angular position control.


TODO:
  Implement yaw rate controller.
  
