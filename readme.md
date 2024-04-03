# QEnv
## Introduction
This Python-based project is designed as an extension to the capabilities of the original QCarSteeringControl project. It focuses on enabling remote control of Quanser Devices and control within the QLab environment. While it builds upon the ideas of the original project, it operates independently and is not backward compatible.

## Supported Devices
### Logitech
This project was tested with the Logitech G920 Driving Force Racing Wheel controller, but according to Logitech document, it should also work with the following devices:
- G29
- G920
- Driving Force GT
- G27
- G25
- Driving Force
- Formula Force GP
- Formula Force
### Quanser
- Quanser QCar

## Installation
To install and run this project, you need to have some packages, QLab and Logitech G Hub installed on your workstation.
- third party requirements: `pip install -r requirements.txt`
- Logitech G Hub: https://www.logitechg.com/en-ca/innovation/g-hub.html
- quanser package and QLab are provided by Quanser

## Usage
You can choose to only control QCar in the virtual environment or control the QCar in both virtual environment and the real world.

For Logitech steering wheel controller
- Steering:
  <br>Turn the steering wheel left or right to steer the QCar in the respective direction.
- Throttle:
  <br>Gently press the accelerator pedal on the Logitech steering wheel controller to increase speed. You can stop the QCar by release the pedal.
- Cruise Control:
  <br>Press Down button on the steering wheel controller to engage and disengage cruise control, maintaining a steady speed.
- Reverse:
  <br>Press Up button on the steering wheel controller to enable and diable the reverse function.
- Toggle Lights On/Off:
  <br>Press A button on the steering wheel controller to turn the QCar's light on and off, enhancing visibility during different conditions.
- Safe Mode:
  <br>Press XBOX button on the steering wheel controller to lock/unlock the QCar.

<br>For Keyboard controller
- Steering:
  <br>Press 'A' or 'D' to steer the QCar in the respective direction.
- Throttle:
  <br>Press 'W' to increase speed. You can stop the QCar by pressing 'S'.
- Cruise Control:
  <br>Press 'Q' to engage and disengage cruise control, maintaining a steady speed.
- Reverse:
  <br>Press 'E' to enable and diable the reverse function.
- Toggle Lights On/Off:
  <br>Press 'L' to turn the QCar's light on and off, enhancing visibility during different conditions.
- Safe Mode:
  <br>Press '/' to lock/unlock the QCar.

## License
This project is licensed under the Apache-2.0 license - see the LICENSE file for details.