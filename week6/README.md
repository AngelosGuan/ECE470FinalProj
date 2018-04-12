# ECE 470 Introduction to Robotics : Demonstrate Path Planning
 Group members: **Yite Wang** (yitew2@illinois.edu), **Angelos Guan** (tguan2@illinois.com)
### 1. Introduction
The purpose of the group this week is to demonstrate path planning of the chosen robot.
### 2. Tools
`V-REP` is used for simulation. Robot `UR3` is used for the simulation. Code is writen in `Python` to control the robot in the `V-REP`.
### 3. Detailed steps for reproducing the result
Result video which is recorded in MacOS operating system can be found [here](https://youtu.be/4Zv0HvncjgA).

**Note: The following steps are conducted in Windows 10 operating system.**

Here are the steps for reproducing the result.
- ##### 3.1 Download and install V-REP
  - Download `V-REP` first. The `V-REP` non-limited educational version can be found and downloaded [here](http://www.coppeliarobotics.com/downloads.html).
  - Finish downloading the `V-REP` software and install it in your computer. `V-REP` is installed in the system disk in default.
  - After finishing the installation, open `V-REP PRO EDU` on your desktop.
- ##### 3.2 Open the document
  - Click `File-Open Scene`, choose the `fifth(410).ttt` file. The UR3 bot and a box representing obstacle in environment will be shown.
  - Make sure that python is installed. Create a new folder outside of `V-REP` (e.g., vrep_test). Copy these files from `V-REP` installation directory into the created folder: (Note: This step will not be shown in the video.)
  ```
  vrep/programming/remoteApiBindings/python/python/vrep.py
  vrep/programming/remoteApiBindings/python/python/vrepConst.py
  vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
  ```
  - Download and put the file which contains our code in the folder as well:
  ```
  pp.py
  ```

- ##### 3.3 Observe the path planning.
  - Start `Anaconda Prompt` from your computer and run this code with this command (inside the folder vrep_test):
  ```
  python pp.py
  ```
  - Input two set of theta representing start pose and end pose on the command line of the terminal. In the GUI, you should see the process of generating a path from start pose to goal pose. In addition, after reaching the goal pose, the robot would then plan a path back to the initial pose.
  **Note: There is no path planning from the original position to the starting position.**


### 4. Beyond the minimum requirement
- ##### 4.1 User interface
  - User interface is constructed to add user-define obstacles in the scene.
- ##### 4.2 Robot reset
  - Path is also planned while resetting the robot from the end position to the original position.
