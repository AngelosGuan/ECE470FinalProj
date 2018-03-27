# ECE 470 Introduction to Robotics : Demonstrate Inverse Kinematics

 Group members: **Yite Wang** (yitew2@illinois.edu), **Angelos Guan** (angelosguan@gmail.com)

### 1. Introduction

The purpose of the group this week is to demonstrate inverse kinematics of the chosen robot.


### 2. Tools

`V-REP` is used for simulation. Robot `UR3` is used for the simulation. Code is writen in `Python` to control the robot in the `V-REP`.


### 3. Detailed steps for reproducing the result

Result video can be found
[here](https://youtu.be/E0LEY2i6P9I).

**Note: The following steps are conducted in Windows 10 operating system.**

Here are the steps for reproducing the result.

- ##### 3.1 Download and install V-REP

  - Download `V-REP` first. The `V-REP` non-limited educational version can be found and downloaded [here](http://www.coppeliarobotics.com/downloads.html).

  - Finish downloading the `V-REP` software and install it in your computer. `V-REP` is installed in the system disk in default.

  - After finishing the installation, open `V-REP PRO EDU` on your desktop.

- ##### 3.2 Open the document

  - Click `File-Open Scene`, choose the `fourth_week.ttt` file. The UR3 robot with a `reference frame` will be shown. This reference frame represents the `Destination`.

  - Make sure that python is installed. Create a new folder outside of `V-REP` (e.g., vrep_test). Copy these files from `V-REP` installation directory into the created folder: (Note: This step will not be shown in the video.)
  ```
  vrep/programming/remoteApiBindings/python/python/vrep.py
  vrep/programming/remoteApiBindings/python/python/vrepConst.py
  vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
  ```

  - Download and put the file which contains our code in the folder as well:
  ```
  in.py
  ```

- ##### 3.3 Set the Destination of the robot.
    - First left-click the reference frame. Then click `Object / item shift` icon on the top of the GUI. A small window will pop up.

    - Make sure `Mouse Translation` - `Preferred axes` is set to your preferred orientation.

    - Use mouse to drag the reference frame to have the goal destination.

    - Click `Object / item rotate` icon on the top of the GUI. A small window will pop up.

    - Make sure `Mouse Rotation` - `Preferred axis` is set to your preferred axis.

    - Use mouse to drag the reference frame to have the goal orientation of the frame.

- ##### 3.4 Observe the inverse kinematics.

  - Start `Anaconda Prompt` from your computer and run this code with this command (inside the folder vrep_test):
  ```
  python in.py
  ```
  - In the GUI, you should see the process of the calculation and finally `UR3` robot will move to the destination if no mistake takes place. [**Note: If the robot can not reach the destination, you will see warning in the terminal.**]



### 4. Beyond the minimum requirement

- ##### 4.1 Visualization of the calculation process
  When conducting section 3.4, a process of the calculation will be shown, which gives the user a sense of whether the computer is working or the robot can not reach the destination.
  (This part is commented out in the code in order to speed up the calculation.)

- ##### 4.2 Add more user-specified determination points
  Add two more reference frame in the scene. The robot would access these references one by one. The user can drag these frames around in the vrep to change the target position for the robot. A smoke effect will indicate the current goal frame. If a frame cannot be reached by the robot, an error message would be print to the terminal.

- ##### 4.3 Adding the smoke visual effect.
  Add a smoke object into the scene. Get its handle in the code. Change its position to the same position as the current reference frame we are trying to access.



**It should be noticed that actually in the demo, the joint didn't move to its positive and negative limit because the joint will hit the ground or other part of the robot. So we only made each joint of the robot move to part of its limit. **
