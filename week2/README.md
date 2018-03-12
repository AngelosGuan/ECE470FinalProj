# ECE 470 Introduction to Robotics : Demonstrate robot motion with code in simulator

 Group members: **Yite Wang** (yitew2@illinois.edu), **Angelos Guan** (angelosguan@gmail.com)

### 1. Introduction

The purpose of the group this week is to demonstrate robot motion with code in simulator. 


### 2. Tools

`V-REP` is used for simulation. Robot `UR3` is used for the simulation. Code is writen in `Python` to control the robot in the `V-REP`. 


### 3. Detailed steps for reproducing the result

Result video can be found [here](https://www.youtube.com/watch?v=YVZi6rSPHNM). 

**Note: The following steps are conducted in Windows 10 operating system.**

Here are the steps for reproducing the result.

- Download `V-REP` first. The `V-REP` non-limited educational version can be found and downloaded [here](http://www.coppeliarobotics.com/downloads.html). 

- Finish downloading the `V-REP` software and install it in your computer. `V-REP` is installed in the system disk in default.

- After finishing the installation, open `V-REP PRO EDU` on your desktop.

- We will use a robot base then choose a hand among various hand models to do our simulation.

	- In `Model browser` window on the left hand side of the GUI, click on `robots -> non-mobile`, a list of various robots are shown in the bottom left corner of the software.

	- Find robot `UR3.ttm`. Drag the robot to the center area of the software. It can be seen that there is a robot in the scene.

	- Under `components -> grippers`, find `Jaco hand.ttm` and drag it to the scene.

- Now, the `Jaco hand` will be installed to `link 7` of `UR3` robot.

	- Under `Scene hierarchy` window, look for `new scene(scene 1)`-`UR3`, expand `UR3` by clicking `+` sign next to it. A list of joints and links will be shown under `UR3`.

	- Click `UR3_link7`-`UR3_connection`. Hold `Ctrl` and click on `JacoHand` under `Scene hierarchy`-`new scene(scene 1)`.

	- With `UR3_connection` and `JacoHand` selected, find and click ![Imgur](https://i.imgur.com/xdPXhvu.png) `Assemble / Disassemble` icon on the top of the software. You can find that the gripper and the robot are connected.

- Still in the `Scene hierarchy` window, double-lick the ![Imgur](https://i.imgur.com/xIkoL8n.png) icon on the right of the `UR3`. A window which contains `Threaded child script` will pop out. Delete all the example code in this file, we will run our own code.

- Make sure that python is installed. Create a new folder outside of `V-REP` (e.g., vrep_test). Copy these files from `V-REP` installation directory into the created folder:
```
vrep/programming/remoteApiBindings/python/python/vrep.py
vrep/programming/remoteApiBindings/python/python/vrepConst.py
vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
```

- Download and put the file which contains our code in the folder as well:
```
test.py
```
- Start `Anaconda Prompt` from your computer and run this code with this command (in side the folder vrep_test):
```
python test.py
```
- In the GUI, you should see `UR3` robot move its each joint. 

Also, check [here](https://www.youtube.com/watch?v=9960STJputM&feature=youtu.be) for video which shows the same procedures in MacOS system.


### 4. Finding the limit of each joint.

In the robot simulation, finding the limit for each joint is important because giving positions where a specific joint can not reach will **do harm to** the robot. Here is the way to find joint limit for each joint.

- Choose and click the joint whom you want to find the limit of (for example `joint1`) under `Scene hierarchy`-`new scene(scene 1)`-`UR3`. 

- Click `Tools`-`Scene Object Properties`.

- You will find the information for joint limit in `configuration` part.

- If box on the left of `Position is cyclic` is ticked, then it indicates that the joint is cyclic. It can varie between -180 and +180 degrees without limitation.

- If `Position is cyclic` is not ticked, you can find the `position minimum` and `Position range`.

**It should be noticed that actually in the demo, the joint didn't move to its positive and negative limit because the joint will hit the ground or other part of the robot. So we only made each joint of the robot move to part of its limit. **