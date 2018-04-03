import vrep



import time



import numpy as np



import scipy.linalg as sla











def skew3(arr):



    a=arr[0]



    b=arr[1]



    c=arr[2]



    mat = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])



    return mat







def skew6(arr):



    a = arr[0]



    b = arr[1]



    c = arr[2]



    d = arr[3]



    e = arr[4]



    f = arr[5]



    mat = np.array([[0,-c,b,d],[c,0,-a,e],[-b,a,0,f],[0,0,0,0]])



    return mat







def getS(a,q):



    a1=a[0]



    a2=a[1]



    a3=a[2]



    b=-skew3(a).dot(q)



    b1 = b[0]



    b2=b[1]



    b3=b[2]



    mat = np.array([a1,a2,a3,b1,b2,b3])[:,None]



    return mat







def exm(s,theta):



    return sla.expm(skew6(s)*theta)



def forward_k(joints, theta,clientID):







    s = []



    a = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[0,0,1],[0,1,0]]);



    x = 0



    y = 0



    z = 0



    vector = [[0],[0],[0]]



    for i in range(0,6):



    	result,vector=vrep.simxGetObjectPosition(clientID,joints[i],joints[0],vrep.simx_opmode_blocking)



    	if result != vrep.simx_return_ok:



    		raise Exception('could not get position')



    	q=np.array([[vector[0]],[vector[1]],[vector[2]]])



    	s.append(getS(a[i],q)[:,0])



    s=np.array(s).T



    result,vector=vrep.simxGetObjectPosition(clientID,joint_end,joints[0],vrep.simx_opmode_blocking)

    result,theta_come =vrep.simxGetObjectQuaternion(clientID,joint_end,joints[0],vrep.simx_opmode_blocking)



    if result != vrep.simx_return_ok:



    	raise Exception('could not get object orientation')



    qx=theta_come[0]



    qy=theta_come[1]



    qz=theta_come[2]



    qw=theta_come[3]



    #M = [[np.cos(t2)*np.cos(t1), np.sin(t3)*np.sin(t2)*np.cos(t1)-np.cos(t3)*np.sin(t1),np.cos(t3)*np.sin(t2)*np.cos(t1)+np.sin(t3)*np.sin(t1),vector[0]],



    #[np.cos(t2)*np.sin(t1),np.sin(t3)*np.sin(t2)*np.sin(t1)+np.cos(t3)*np.cos(t1),np.cos(t3)*np.sin(t2)*np.sin(t1)-np.sin(t3)*np.cos(t1),vector[1]],



    #[-np.sin(t2),np.sin(t3)*np.cos(t2),np.cos(t3)*np.cos(t2),vector[2]],



    M= np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, vector[0]],



    [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, vector[1]],



    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, vector[2]],



    [0,0,0,1]])

    T_test = exm(s[:,0],theta[0]).dot(exm(s[:,1],theta[1]))



    T_test= T_test.dot(exm(s[:,2],theta[2]))



    T_test = T_test.dot(exm(s[:,3],theta[3]))



    T_test = T_test.dot(exm(s[:,4],theta[4]))



    T_test = T_test.dot(M)



    T = exm(s[:,0],theta[0]).dot(exm(s[:,1],theta[1]))



    T = T.dot(exm(s[:,2],theta[2]))



    T = T.dot(exm(s[:,3],theta[3]))



    T = T.dot(exm(s[:,4],theta[4]))



    T = T.dot(exm(s[:,5],theta[5]))



    T = T.dot(M)



    return T















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







result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)



if result != vrep.simx_return_ok:



    raise Exception('could not get object handle for sixth joint')







result, joint_end = vrep.simxGetObjectHandle(clientID, 'UR3_connection', vrep.simx_opmode_blocking)



if result != vrep.simx_return_ok:



    raise Exception('could not get object handle for sixth joint')







jointHandles = []



jointHandles.append(joint_one_handle)



jointHandles.append(joint_two_handle)



jointHandles.append(joint_three_handle)



jointHandles.append(joint_four_handle)



jointHandles.append(joint_five_handle)



jointHandles.append(joint_six_handle)











# Start simulation



vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)







# Wait 0.5 seconds



time.sleep(0.5)





for i in range(0,3):

    for x in range(0,6):

        vrep.simxSetJointTargetPosition(clientID, jointHandles[x], 0, vrep.simx_opmode_oneshot)

    user_input = input("please input 6 angles separated by ,")

    input_list = user_input.split(',')

    theta = [float(z) for z in input_list]
    
    test_limit=0

    for j in range(0,5):
    	if theta[j]<-np.pi or theta[j]>2*np.pi:
    		test_limit=1

    if test_limit == 1:
    	print('######################################')
    	print('##### Violating joint limit! #########')
    	print('######################################')
    vrep.simxSetJointTargetPosition(clientID, jointHandles[0], theta[0], vrep.simx_opmode_oneshot)

    time.sleep(0.5)

    vrep.simxSetJointTargetPosition(clientID, jointHandles[1], theta[1], vrep.simx_opmode_oneshot)

    time.sleep(0.5)

    vrep.simxSetJointTargetPosition(clientID, jointHandles[2], theta[2], vrep.simx_opmode_oneshot)

    time.sleep(0.5)

    vrep.simxSetJointTargetPosition(clientID, jointHandles[3], theta[3], vrep.simx_opmode_oneshot)

    time.sleep(0.5)

    vrep.simxSetJointTargetPosition(clientID, jointHandles[4], theta[4], vrep.simx_opmode_oneshot)

    time.sleep(0.5)

    vrep.simxSetJointTargetPosition(clientID, jointHandles[5], theta[5], vrep.simx_opmode_oneshot)


    time.sleep(2)

 #   err, contact = vrep.simxGetCollisionHandle (clientID, 'Collision2' ,vrep.simx_opmode_blocking)

 #   err, collision = vrep.simxReadCollision (clientID, contact , vrep.simx_opmode_streaming)
    
    return_code,state_self=vrep.simxGetIntegerSignal(clientID, "co_self", vrep.simx_opmode_blocking)
    
    return_code,state_ground =vrep.simxGetIntegerSignal(clientID, "co_ground", vrep.simx_opmode_blocking)

    return_code,state_box =vrep.simxGetIntegerSignal(clientID, "co_box", vrep.simx_opmode_blocking)

    time.sleep(0.5)

    if state_self:
    	print('######################################')
    	print('#### Collide with robot itself! ######')
    	print('######################################')

    if state_ground:
    	print('######################################')
    	print('###### Collide with the ground! ######')
    	print('######################################')

    if state_box:
    	print('######################################')
    	print('###### Collide with the cylinder! ####')
    	print('######################################')
    if state_self == 0 and state_ground == 0 and state_box == 0:
    	print('######################################')
    	print('############ No Collision~ ###########')
    	print('######################################')

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)







# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):



vrep.simxGetPingTime(clientID)







# Close the connection to V-REP



vrep.simxFinish(clientID)

