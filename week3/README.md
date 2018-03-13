# ECE 470 Introduction to Robotics : Demonstrate forward kinematics in a simulator

 Group members: **Yite Wang** (yitew2@illinois.edu), **Angelos Guan** (tguan2@illinois.edu)

### 1. Introduction

The goal this week is to demonstrate forward kinematics in a simulator.


### 2. Tools

`V-REP` is used for simulation. Robot `UR3` is used for the simulation. Code is writen in `Python` to control the robot in the `V-REP`.


### 3. Detailed steps for reproducing the result

Result video in Windows10 operating system can be found [here](https://youtu.be/VC3IMedB2OE).

**Note: The following steps are conducted in MacOS operating system.**

Here are the steps for reproducing the result.

- Download `V-REP` first. The `V-REP` non-limited educational version can be found and downloaded [here](http://www.coppeliarobotics.com/downloads.html).

- Finish downloading the `V-REP` software and install it in your computer. `V-REP` is installed in the system disk in default.

- After finishing the installation, open `V-REP PRO EDU` on your desktop.

- We will use UR3 to do our forward kinematics simulation.

  - Click `File-Open Scene`, choose the `second_week.ttt` file. The UR3 robot with a dummy object attached to `UR3_connection`. This object represents the tool frame. Note that the child script of the robot has been deleted.

	- Alternatively, the given scene can be reached step by step as below.

   - In `Model browser` window on the left hand side of the GUI, click on `robots -> non-mobile`, a list of various robots are shown in the bottom left corner of the software.

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
final_test.py
```
- Make sure the `ece470.yml` environment is set correctly. Type `source activate ece470` in terminal first to set the environment. Then run this code with following command (in side the folder):
```
python final_test.py
```
- In the GUI, you should see `UR3` robot move according to the input theta. Predicted frame will be displayed in the scene. It can be shown that the current tool frame will finally fit the predicted frame. In the terminal, the predicted tool-frame pose using forward kinematics and the actual pose of tool-frame achieved by passing in theta to `simxSetObjectPosition()` should be printed.


### 4. Code

- For Calculating tool pose using theta, we choose the product of exponential method using the `forward_k()` function. The `forward_k()` takes in an array of joint handle which contains all six joints of the robot, an array of thetas, and clientID of the connection to simulator.

- In the `forward_k()` function, firstly, calculate spatial twist of each joint by the following equation.
`-[a]*q`
Obtain `a` by looking at each joint and get an approximate orientation vector, and `q` by the returned vector of `simxGetObjectPosition()`.

- Secondly, calculate the initial pose M by obtaining the quaternion using the function `simxGetObjectQuaternion()`. Obtain the rotation matrix by transforming the quaternion to a rotation matrix and get the relative position by `simxGetObjectPosition()`.

```
  M= np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, vector[0]],
              [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, vector[1]],
              [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, vector[2]],
              [0,0,0,1]])
```

- Thirdly, calculate the initial pose using the equation from product of exponentials.

```
  T_test = exm(s[:,0],theta[0]).dot(exm(s[:,1],theta[1]))
  T_test= T_test.dot(exm(s[:,2],theta[2]))
  T_test = T_test.dot(exm(s[:,3],theta[3]))
  T_test = T_test.dot(exm(s[:,4],theta[4]))
  T_test = T_test.dot(M)
```

- For the main part of program, firstly we set the joint1 frame as the referenceFrame for all the other joints.

```
result,frame=vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)
change_frame_v=vrep.simxSetObjectPosition(clientID,frame,jointHandles[0],coordinate,vrep.simx_opmode_oneshot)
```

- Before we actually move the robot, use the `forward_k()` to predict the tool frame after transformation using given theta.
Then, use `simxSetObjectPosition()` to move the robot using given theta.
Lastly, use `simxSetObjectQuaternion()` to move the tool frame to the exact same place as predicted. (Going beyond requirements.)

### 5. beyond requirements: Adding a dummy object to represent the tool frame.

#### 5.1 Line visualization
In the project, we draw a line to link the calculated pose and current pose of the connection of the UR3 robot.

Here is the way to do such thing.
- First, make sure `V-REP` is currently working. Click the dummy or a frame, click `Tools`-`Scene Object Properties` on the top of the `V-REP` GUI.
- Choose desired object in `Linked Dummy` blank.

#### 5.2 User Interface

We created an user interface to facilitate users to input six theta values for six joints. We wrapped the whole main function in the for loops and use simple input function in python as follows:
```
    user_input = input("please input 6 angles separated by ,")
    input_list = user_input.split(',')
    theta = [float(z) for z in input_list]

```

#### 5.3 Getting position and orientation using API function

For all the orientation and position (except orientation of each joint), we used the API functions to determine relative positions of different joints to base frame instead of using sketch.
