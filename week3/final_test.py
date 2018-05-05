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

    #theta = [np.pi,2*np.pi/6,np.pi/6,np.pi/6,np.pi/6,np.pi/6]



    #theta=[np.pi/4,0,np.pi/2,0,np.pi/6,np.pi/3]



    #theta=[0,0,0,0,0,np.pi/3]







    T_pre = forward_k(jointHandles, theta,clientID)



    x1=T_pre[0,3]



    y1=T_pre[1,3]



    z1=T_pre[2,3]



    coordinate=np.array([x1,y1,z1])



    R=T_pre[0:3,0:3]



    w_vector=sla.logm(R)



    m1=w_vector[2,1]



    m2=w_vector[0,2]



    m3=w_vector[1,0]



    a_pre=np.array([[m1],[m2],[m3]])



    n=sla.norm(a_pre)



    alpha=np.array([[m1/n],[m2/n],[m3/n]])



    theta_forq=sla.norm(a_pre)



    w_new=np.cos(theta_forq/2)



    q1_new=alpha[0]*np.sin(theta_forq/2)



    q2_new=alpha[1]*np.sin(theta_forq/2)



    q3_new=alpha[2]*np.sin(theta_forq/2)



    new_orientation=[q1_new,q2_new,q3_new,w_new]



    #result,dummy=vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)



    #change_dummy=vrep.simxSetObjectPosition(clientID,dummy,jointHandles[0],coordinate,vrep.simx_opmode_oneshot)



    result,frame=vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)



    change_frame_v=vrep.simxSetObjectPosition(clientID,frame,jointHandles[0],coordinate,vrep.simx_opmode_oneshot)



    change_frame_theta=vrep.simxSetObjectQuaternion(clientID,frame,jointHandles[0],new_orientation,vrep.simx_opmode_oneshot)



    time.sleep(1)







    ##################################################################################################################################



    ########################################### Now just let the first joint move ####################################################



    ##################################################################################################################################







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



    result,vector=vrep.simxGetObjectPosition(clientID,jointHandles[5],jointHandles[0],vrep.simx_opmode_blocking)




    result2,theta_come2 =vrep.simxGetObjectQuaternion(clientID,jointHandles[5],jointHandles[0],vrep.simx_opmode_blocking)




    qx=theta_come2[0]



    qy=theta_come2[1]



    qz=theta_come2[2]



    qw=theta_come2[3]



    M_test= np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, vector[0]],



    [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, vector[1]],



    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, vector[2]],



    [0,0,0,1]])



    #print('after first move ')



    #print(M_test)







    ##################################################################################################################################



    ########################################## M_test Matrix here is wrong ###########################################################



    ##################################################################################################################################







    ##for x in range(1,6):



    ##    vrep.simxSetJointTargetPosition(clientID, jointHandles[x], theta[x], vrep.simx_opmode_oneshot)



    ##    time.sleep(1)











    result,vector=vrep.simxGetObjectPosition(clientID,joint_end,jointHandles[0],vrep.simx_opmode_blocking)







    result2,theta_come2 =vrep.simxGetObjectQuaternion(clientID,joint_end,jointHandles[0],vrep.simx_opmode_blocking)











    #T_real = forward_k(jointHandles, [0,0,0,0,0,0],clientID)







    qx=theta_come2[0]



    qy=theta_come2[1]



    qz=theta_come2[2]



    qw=theta_come2[3]



    M1 = np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, vector[0]],



    [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, vector[1]],



    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, vector[2]],



    [0,0,0,1]])



    print('current value of Tool Pose = ')



    print(M1)



    print('calculated value of Tool Pose = ')



    print(T_pre)







# Stop simulation



vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)







# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):



vrep.simxGetPingTime(clientID)







# Close the connection to V-REP



vrep.simxFinish(clientID)

