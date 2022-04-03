# Resources and test files/modules

This repository contains separeted files (called "subsystems" in FRC) separated by category. The file will depend on the type of test the team would like to perform on the robot.
<br>**All files have been written using <a href="https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/TimedRobot.html">TimedRobot</a> class**.
## Avaliable Teleoperated Tests
- Pneumatics
- Drivetrain (Arcade Drive)
- Single Motor Controller
## Others Tests
- Physics (Only for Robot Simulator)
## Deploy
For more information, <a href="https://robotpy.readthedocs.io/en/stable/guide/deploy.html">click here.</a><br>
Replace 'name_subsystem' to folder that you would like to access.
### Windows
- Executing Robot Simulator
```
cd 'name_subsystem'
py -3 robot.py sim
```
- Deploy to the robot
```
cd 'name_subsystem'
py -3 robot.py deploy
```
---
### Linux/macOS
- Executing Robot Simulator
```
cd 'name_subsystem'
python3 robot.py sim
```
- Deploy to the robot
```
cd 'name_subsystem'
python3 robot.py deploy
```
## Using Robot Simulator (Reminder highlighted in red)
### Do not forget to enable a robot state!<br>
![robot_state](https://user-images.githubusercontent.com/73722088/161405945-4b1c07eb-d35c-4ab3-ae5f-e23df67c188e.png)
## Credits
All the commands are from RobotPy (Python 3 to the FRC).<br>
**<a href="https://robotpy.readthedocs.io/en/stable/index.html">Click here to open documentation</a>**
