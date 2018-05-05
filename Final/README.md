# ECE 470 Introduction to Robotics Final Project: PARCEL SORTING

 Group members: **Yite Wang** (yitew2@illinois.edu), **Angelos Guan** (tguan2@illinois.com)

### 1. INTRODUCTION

All the documents contained in the RAR file is the final projcet of ECE 470. In the final project, the robot will finally successfully detects three different colors of different objects. The robot will grab all the objects successfully and sort them into different buckets according to their colors.

### 2. METHODOLOGY

`V-REP` is used for simulation. Robot `UR3` is used for the simulation. Gripper `BarrettHand` is connected to `UR3` robot to make it possible for the robot to grab objects. `CircularConveyorBelt` is added to the scene to simulate real conveyor. Code is writen in `Python` to control the robot in the `V-REP`.

### 3. DETAILED STEPS FOR REPRODUCING THE RESULT

Result video which is recorded in MacOS operating system can be found [here](https://www.youtube.com/watch?v=A9oWJg_2RLE&index=28&list=PLAfFM-5AffXRPtUbenrsllKjKWfReC0cx&).



**Note: The following steps are conducted in Windows 10 operating system.**



Here are the steps for reproducing the result.

- ##### 3.1 Download and install V-REP

  - Download `V-REP` first. The `V-REP` non-limited educational version can be found and downloaded [here](http://www.coppeliarobotics.com/downloads.html).

  - Finish downloading the `V-REP` software and install it in your computer. `V-REP` is installed in the system disk in default.

  - After finishing the installation, open `V-REP PRO EDU` on your desktop.

- ##### 3.2 Open the document

  - Click `File-Open Scene`, choose the `final_scene.ttt` file. The UR3 bot and all the other objects are shown in the scene.

  - Make sure that python is installed. Create a new folder outside of `V-REP` (e.g., vrep_test). Copy these files from `V-REP` installation directory into the created folder: (Note: This step will not be shown in the video.)
  ```
  vrep/programming/remoteApiBindings/python/python/vrep.py
  vrep/programming/remoteApiBindings/python/python/vrepConst.py
  vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
  ```

  - Download and put the file which contains our code in the folder as well:
  ```
  final.py
  ```



- ##### 3.3 Observe the `Parcel Sorting`.

  - Start `Anaconda Prompt` from your computer and run this code with this command (inside the folder vrep_test):
  ```
  final.py
  ```
  - You will be able to see the robot detecting all the objects moving on the conveyor belt and then sort all the objects to corresponding buckets according to its color.

 


### 4. ACKNOWLEDGMENT

We would like to thank our instructor Prof. Bretl at University of Illinois at Urbana-Champaign. He was always willing to answer our questions when we were in trouble. We would also like to thank our lab teaching assistant Siwei Tang, without whose help we would not be able to solve difficult problems one by one. 