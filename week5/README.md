# ECE 470 Introduction to Robotics : Demonstrate Collision Detection

 Group members: **Yite Wang** (yitew2@illinois.edu), **Angelos Guan** (tguan2@illinois.com)

### 1. Introduction

The purpose of the group this week is to demonstrate collision detection of the chosen robot.

### 2. Tools

`V-REP` is used for simulation. Robot `UR3` is used for the simulation. Code is writen in `Python` to control the robot in the `V-REP`.

### 3. Detailed steps for reproducing the result

Result video which is recorded in MacOS operating system can be found [here](https://youtu.be/3CcSuqGpM7U).



**Note: The following steps are conducted in Windows 10 operating system.**



Here are the steps for reproducing the result.

- ##### 3.1 Download and install V-REP

  - Download `V-REP` first. The `V-REP` non-limited educational version can be found and downloaded [here](http://www.coppeliarobotics.com/downloads.html).

  - Finish downloading the `V-REP` software and install it in your computer. `V-REP` is installed in the system disk in default.

  - After finishing the installation, open `V-REP PRO EDU` on your desktop.

- ##### 3.2 Open the document

  - Click `File-Open Scene`, choose the `fourth_week.ttt` file. The UR3 bot and a box representing obstacle in environment will be shown.

  - Make sure that python is installed. Create a new folder outside of `V-REP` (e.g., vrep_test). Copy these files from `V-REP` installation directory into the created folder: (Note: This step will not be shown in the video.)
  ```
  vrep/programming/remoteApiBindings/python/python/vrep.py
  vrep/programming/remoteApiBindings/python/python/vrepConst.py
  vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
  ```

  - Download and put the file which contains our code in the folder as well:
  ```
  co.py
  ```



- ##### 3.3 Observe the collision detection.

  - Start `Anaconda Prompt` from your computer and run this code with this command (inside the folder vrep_test):
  ```
  python co.py
  ```

  - Input a set of thetas on the command line of the terminal. In the GUI, you should see the process of the collision detection and finally the result would be printed to terminal and indicated in GUI by change of color.


### 4. Beyond the minimum requirement
- ##### 4.1 Visualization of collision
  Visualization of collision is realized by showing contour of different colors in `V-REP`. 
    - When there is collision between the `Robot` and `Robot` itself, then `RED` contour will show the place where collision occurs
    - When there is collision between the `Robot` and `Ground` , then `BLUE` contour will show the place where collision occurs
    - When there is collision between the `Robot` and `Box` (or obstacle) , then `GREEN` contour will show the place where collision occurs
- ##### 4.2 User interface
  We created an user interface to facilitate users to input six theta values for six joints.
- ##### 4.3 Output
  Our program will show different kinds of results through command window.
