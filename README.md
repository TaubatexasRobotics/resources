# Resources and test files/modules

This repository contains files separated by category. The file will depend on the type of test the team would like to perform on the robot.
<br>**All files were written using <a href="https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/TimedRobot.html">TimedRobot</a> class**.
## Setup & Installation
For more information about installing RobotPy's packages, <a href="https://robotpy.readthedocs.io/en/stable/install/index.html">click here.</a><br>
### Prerequisites
After cloning the repository, you must have <a href="https://www.python.org/downloads/">Python 3</a> and <a href="https://pypi.org/project/pip/">pip</a> installed on your computer.
### Installing packages
```
pip install -r requirements.txt
```
## Testing code
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

## Contributions

Please check out the <a href="https://github.com/robotpy/examples/blob/main/CONTRIBUTING.md">RobotPy's porting guide</a> for a variety of guidelines for new examples.
