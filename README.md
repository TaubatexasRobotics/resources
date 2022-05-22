# Resources and test files/modules

This repository contains files separated by category. The file will depend on the type of test the team would like to perform on the robot.
<br>**All files were written using <a href="https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/TimedRobot.html">TimedRobot</a> class**.
## Avaliable Teleoperated Tests
- Pneumatics;
- Drivetrain (Arcade Drive with CTRE Victor SPX);
- Single Motor Controller (CTRE Victor SPX).
## Others Tests
- Physics (Only for Robot Simulator);
- Serial UART (Communication between Arduino and RoboRIO).
## Deploy
For more information, <a href="https://robotpy.readthedocs.io/en/stable/guide/deploy.html">click here.</a><br>
Replace 'name_test' to folder that you would like to access.
### Windows
- Executing Robot Simulator
```
cd 'name_test'
py -3 robot.py sim
```
- Deploy to the robot
```
cd 'name_test'
py -3 robot.py deploy
```
---
### Linux/macOS
- Executing Robot Simulator
```
cd 'name_test'
python3 robot.py sim
```
- Deploy to the robot
```
cd 'name_test'
python3 robot.py deploy
```
## Using Robot Simulator (Reminder highlighted in red)
### Do not forget to enable a robot state!<br>
![Screenshot 2022-05-22 004139](https://user-images.githubusercontent.com/73722088/169677573-44665203-6c40-4d09-a6f8-7b2e23cbed30.png)
## Credits
All the commands are from RobotPy (Python 3 to the FRC).<br>
**<a href="https://robotpy.readthedocs.io/en/stable/index.html">Click here to open documentation</a>**
