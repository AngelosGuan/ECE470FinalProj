import vrep
import time
import numpy as np
import scipy.linalg as sla

M=[]
s=[]

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

def untw(arr):
    a=arr[2,1]
    b=arr[0,2]
    c=arr[1,0]
    d=arr[0,3]
    e=arr[1,3]
    f=arr[2,3]
    mat = np.array([[a],[b],[c],[d],[e],[f]])
    return mat

def ad(arr):
    R=arr[0:3,0:3]
    p=arr[0:3,3]
    A=np.kron(np.eye(2),R)
    B=np.kron(np.array([[0,0],[1,0]]),skew3(p).dot(R))
    mat=A+B
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

    #######################################################################################################
    ################################# get positions for all joints ########################################
    #######################################################################################################

    result,vector=vrep.simxGetObjectPosition(clientID,joint_end,joints[0],vrep.simx_opmode_blocking)
    result,theta_come =vrep.simxGetObjectQuaternion(clientID,joint_end,joints[0],vrep.simx_opmode_blocking)

    ######################get initial Position and orientation for the end joint###########################

    if result != vrep.simx_return_ok:
    	raise Exception('could not get object orientation')

    qx=theta_come[0]
    qy=theta_come[1]
    qz=theta_come[2]
    qw=theta_come[3]

    M= np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, vector[0]],
    [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, vector[1]],
    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, vector[2]],
    [0,0,0,1]])

    # T = exm(s[:,0],theta[0]).dot(exm(s[:,1],theta[1]))
    # T = T.dot(exm(s[:,2],theta[2]))
    # T = T.dot(exm(s[:,3],theta[3]))
    # T = T.dot(exm(s[:,4],theta[4]))
    # T = T.dot(exm(s[:,5],theta[5]))
    # T = T.dot(M)

    #######################################################################################################
    ################## calculate theoretical pose for a set of joint variables ############################
    #######################################################################################################
    return s,M

def inverse_k(s,M,M_destination,clientID):
    #theta=[]
    flag=0
    V=[]
    for i in range(6):

	    V.append([2])    # random V value that doesn't satisfy the result
    #theta=np.array(np.random.rand(6,1))
    theta=np.array([[0.5],[0.5],[0.5],[0.5],[0.5],[0.5]])
    V=np.array(V)

    error=0.005
    start_v=sla.norm(V)
    while (1):
        V_former=V
        T_current = np.eye(4)
        for i in range(6):
            mul=exm(s[:,i],theta[i,0])
            T_current = T_current.dot(mul)
        T_current=T_current.dot(M)

        V_bracket=sla.logm(M_destination.dot(sla.inv(T_current)))   #space twist

        V=untw(V_bracket)
        if ( sla.norm(V) < error ):
            break
        if (np.absolute(sla.norm(V)-sla.norm(V_former)) < 1e-14):
            flag=1
            print('Can not reach the point.')
            return flag,None

        #V1=sla.inv(ad(T_current)).dot(V);
        N = np.eye(4);
        #J = np.array(s[:,0])[:,None]
        J = s[:,0].reshape((6,1))

        for i in [1,2,3,4,5]:           # 6 here is the number of joints
            N = N.dot(exm(s[:,i-1],theta[i-1,0]))

            #J_new=np.array(ad(N).dot(np.array(s[:,i])[:,None]))
            J_new=np.array(ad(N).dot(s[:,i].reshape((6,1))))

            J = np.concatenate((J,J_new),axis=1)
        thetadot=sla.inv(np.transpose(J).dot(J)+0.01*np.eye(6)).dot(np.transpose(J))
        #thetadot=sla.inv(np.transpose(J).dot(J)).dot(np.transpose(J))
        thetadot=thetadot.dot(V)

        theta = theta + thetadot
        #print('Processing: {0:.3f} %.'.format(100-((sla.norm(V))-error)/start_v*100))
    if ((theta[1,0]<-np.pi/2) or (theta[1,0]>np.pi/2)):
        #flag=1
        print('Can not reach the point, the robot may hit the ground')
        #return flag,None
    return flag,theta


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


result, joint_smoke = vrep.simxGetObjectHandle(clientID, 'smoke', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for smoke')

result,frame3=vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame1')
result,frame2=vrep.simxGetObjectHandle(clientID, 'ReferenceFrame1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame2')
result,frame1=vrep.simxGetObjectHandle(clientID, 'ReferenceFrame2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame3')

frameHandles=[]
frameHandles.append(frame1)
frameHandles.append(frame2)
frameHandles.append(frame3)

#######################################################################################################
############################# get all the handles for joints ##########################################
#######################################################################################################

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

for i in range(3):
    print('Try')
    for x in range(0,6):
        vrep.simxSetJointTargetPosition(clientID, jointHandles[x], 0, vrep.simx_opmode_oneshot)
    time.sleep(1)
#######################################################################################################
############################# set all the joints to initial positions #################################
#######################################################################################################

#    user_input = input("please input 6 angles separated by ,")
#    input_list = user_input.split(',')
#    theta = [float(z) for z in input_list]

#######################################################################################################
########################################## get given joint thetas #####################################
#######################################################################################################

    s,M = forward_k(jointHandles,[1,1,1,1,1,1],clientID)

    # x1=T_pre[0,3]
    # y1=T_pre[1,3]
    # z1=T_pre[2,3]
    # coordinate=np.array([x1,y1,z1])
    # R=T_pre[0:3,0:3]
    # w_vector=sla.logm(R)
    # m1=w_vector[2,1]
    # m2=w_vector[0,2]
    # m3=w_vector[1,0]
    # a_pre=np.array([[m1],[m2],[m3]])
    # n=sla.norm(a_pre)
    # alpha=np.array([[m1/n],[m2/n],[m3/n]])
    # theta_forq=sla.norm(a_pre)
    # w_new=np.cos(theta_forq/2)
    # q1_new=alpha[0]*np.sin(theta_forq/2)
    # q2_new=alpha[1]*np.sin(theta_forq/2)
    # q3_new=alpha[2]*np.sin(theta_forq/2)
    # new_orientation=[q1_new,q2_new,q3_new,w_new]

    #result,frame=vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)


    result,destination_position=vrep.simxGetObjectPosition(clientID,frameHandles[i],jointHandles[0],vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, joint_smoke, jointHandles[0], destination_position,vrep.simx_opmode_blocking);

    result,destination_orientation=vrep.simxGetObjectQuaternion(clientID,frameHandles[i],jointHandles[0],vrep.simx_opmode_blocking)


    #######################################################################################################
    #################### Now calculate the correspoding set of joint variables ############################
    #######################################################################################################

    qx=destination_orientation[0]
    qy=destination_orientation[1]
    qz=destination_orientation[2]
    qw=destination_orientation[3]

    M_destination= np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, destination_position[0]],
    [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, destination_position[1]],
    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, destination_position[2]],
    [0,0,0,1]])

    flag,theta=inverse_k(s,M,M_destination,clientID)
    #######################################################################################################
    ####################### set the objective frame to the desired place ##################################
    #######################################################################################################
    if (flag==0):
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


    #######################################################################################################
    ##################################### move all the joints #############################################
    #######################################################################################################

    # result,vector=vrep.simxGetObjectPosition(clientID,joint_end,jointHandles[0],vrep.simx_opmode_blocking)
    # result2,theta_come2 =vrep.simxGetObjectQuaternion(clientID,joint_end,jointHandles[0],vrep.simx_opmode_blocking)

    # qx=theta_come2[0]
    # qy=theta_come2[1]
    # qz=theta_come2[2]
    # qw=theta_come2[3]
    # M1 = np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, vector[0]],
    # [2*qx*qy +2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, vector[1]],
    # [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, vector[2]],
    # [0,0,0,1]])

#######################################################################################################
####################### show actual values of the end joint ###########################################
#######################################################################################################
    # print('current value of Tool Pose = ')
    # print(M1)
    # print('calculated value of Tool Pose = ')
    # print(T_pre)
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
