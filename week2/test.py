import vrep
import time
import numpy as np

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second joint')

result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third joint')

result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')

result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')

result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta + (2*np.pi), vrep.simx_opmode_oneshot)
# Wait two seconds
time.sleep(1)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

time.sleep(1);

#second joint
# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get two joint variable')
print('current value of two joint variable: theta = {:f}'.format(theta))

# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta + (np.pi/2), vrep.simx_opmode_oneshot)
# Wait one seconds
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta, vrep.simx_opmode_oneshot)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta - (np.pi/2), vrep.simx_opmode_oneshot)
time.sleep(1)

vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta, vrep.simx_opmode_oneshot)
time.sleep(1)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get two joint variable')
print('current value of two joint variable: theta = {:f}'.format(theta))


time.sleep(1)

#third joint
result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get three joint variable')
print('current value of three joint variable: theta = {:f}'.format(theta))

# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta + (np.pi/2), vrep.simx_opmode_oneshot)
# Wait two seconds
time.sleep(1)

vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta, vrep.simx_opmode_oneshot)
time.sleep(1)

vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta - (np.pi/2), vrep.simx_opmode_oneshot)
time.sleep(1)

vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta, vrep.simx_opmode_oneshot)
time.sleep(1)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get three joint variable')
print('current value of three joint variable: theta = {:f}'.format(theta))

time.sleep(1)

#fourth joint
result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get four joint variable')
print('current value of four joint variable: theta = {:f}'.format(theta))

# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta + (np.pi/2), vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta, vrep.simx_opmode_oneshot)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta - (np.pi/2), vrep.simx_opmode_oneshot)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta, vrep.simx_opmode_oneshot)
time.sleep(1)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get four joint variable')
print('current value of four joint variable: theta = {:f}'.format(theta))

time.sleep(1)

#fifth joint
result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get five joint variable')
print('current value of five joint variable: theta = {:f}'.format(theta))

# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta + 2*(np.pi), vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(1)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get five joint variable')
print('current value of five joint variable: theta = {:f}'.format(theta))

time.sleep(1)
#sixth joint
result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get six joint variable')
print('current value of six joint variable: theta = {:f}'.format(theta))

# Set the desired value of the first joint variable
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, -(np.pi), vrep.simx_opmode_oneshot)
# Wait two seconds
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, 2*np.pi, vrep.simx_opmode_oneshot)
# Wait two seconds
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta, vrep.simx_opmode_oneshot)
# Wait two seconds
time.sleep(1)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get six joint variable')
print('current value of six joint variable: theta = {:f}'.format(theta))

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
