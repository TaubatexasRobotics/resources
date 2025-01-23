# Resources and test files/modules

This repository contains files separated by category. The file will depend on the type of test you would like to perform on the robot.
## Setup & Installation
For more information about the setup of RobotPy's packages, <a href="https://robotpy.readthedocs.io/en/stable/install/index.html">click here.</a><br>
### Prerequisites
After cloning the repository, you must have <a href="https://www.python.org/downloads/">Python 3</a> and <a href="https://pypi.org/project/pip/">pip</a> installed on your computer.
### Installing packages
### Windows
```
py -3 -m pip install -r requirements.txt
```
### Linux/macOS
```
python3 -m pip install -r requirements.txt
```
## Testing code
For more information, <a href="https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html">click here.</a> Since 2024, you must need to initialize a RobotPy package, and `pyproject.toml`should <a href="https://docs.wpilib.org/en/stable/docs/software/python/pyproject_toml.html">contains the packages</a> to run some examples of this repository, if it is necessary to reinstall it.<br>

### Windows
- Initialize a FRC Python project
```
py -3 -m robotpy init
```
- Executing Robot Simulator
```
py -3 -m robotpy sim
```
- Sync to install/update 3rd party packages
```
py -3 -m robotpy sync
```
- Deploy to the robot
```
py -3 -m robotpy deploy
```
---
### Linux/macOS
- Initialize a FRC Python project
```
python3 -m robotpy init
```
- Executing Robot Simulator
```
python3 -m robotpy sim
```
- Sync to install/update 3rd party packages
```
python3 -m robotpy sync
```
- Deploy to the robot
```
python3 -m robotpy deploy
```
## Using Robot Simulator (Reminder highlighted in red)
### Do not forget to enable a robot state!<br>
![Screenshot 2022-05-22 004139](https://user-images.githubusercontent.com/73722088/169677573-44665203-6c40-4d09-a6f8-7b2e23cbed30.png)

## Contributions
Please check out the <a href="https://github.com/robotpy/examples/blob/main/CONTRIBUTING.md">RobotPy's porting guide</a> for a variety of guidelines for new examples.

## Credits
<a href="https://robotpy.readthedocs.io/en/stable/index.html">RobotPy</a> (Python 3 for the FRC)<br><a href="https://docs.wpilib.org/en/stable/index.html">WPILib</a>
