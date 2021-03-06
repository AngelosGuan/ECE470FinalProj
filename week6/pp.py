import vrep

import time

import numpy as np

import scipy.linalg as sla



class Tree(object):
    left = None
    right = None
    data = None
    parent = None

    def _init_(self):

        self.left = None

        self.right = None

        self.data = None

        self.parent = None



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



def get_all_s(joints, clientID):

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

    ################## calculate spactial twist ############################

    #######################################################################################################

    return s



def get_initial_position(handle, joints, clientID):

    #######################################################################################################

    ################################# get positions for given handle ######################################

    #######################################################################################################

    result,vector=vrep.simxGetObjectPosition(clientID,handle,joints[0],vrep.simx_opmode_blocking)

    ######################get initial Position and orientation for the end joint###########################

    if result != vrep.simx_return_ok:

    	raise Exception('could not get object orientation')

    p= np.array([[vector[0]],

    [vector[1]],

    [vector[2]]])



    return p



# to be complete

def find_all_target_position(S, theta, all_initpos):

    #######################################################################################################

    ######### get all end positions for all the handles for collision detection ###########################

    #######################################################################################################

    N = np.eye(4)

    all_positions=[[],[],[],[],[],[],[]]

    for z in all_initpos[0]:

        all_positions[0].append(z)

    for i in range(6):

        #S_now = S[:,i].reshape(6,1)

        N = N.dot(exm(S[:,i],theta[i]))

        for j in all_initpos[i+1]:

            m = [j[0],j[1],j[2],1]

            m = np.array(m).reshape(4,1)

            j_new = N.dot(m)

            j_new = [j_new[0,0],j_new[1,0],j_new[2,0]]

            all_positions[i+1].append(j_new)

    return all_positions



# to be complete

def check_current_collision(all_positions, all_radius, p_obstacle, r_obstacle):

    is_collision = 0

    p_in_matrix = []

    r_in_matrix = []

    for i in range (7):

        for j in all_positions[i]:
            p_in_matrix.append(j)

        for j in all_radius[i]:

            r_in_matrix.append(j)

    n_of_balls = len(p_in_matrix)

    p_in_matrix = np.array(p_in_matrix)

    r_in_matrix = np.array(r_in_matrix)

    for i in range (3,n_of_balls):
        if p_in_matrix[i,2] <0:
            is_collision =1



    for i in range (n_of_balls-1):

        for j in range (i+1,n_of_balls):



            diff = p_in_matrix[i,:]-p_in_matrix[j,:]


            if sla.norm(diff) < r_in_matrix[i,0]+r_in_matrix[j,0]:




                is_collision = 1
                #print(i,j)

                break

        n_of_obstacles = len(p_obstacle)

        for j in range(n_of_obstacles):


            diff = p_in_matrix[i,:]-p_obstacle[j,:]


            if sla.norm(diff) < r_in_matrix[i,0]+r_obstacle[j,0]:

                is_collision = 1

                break

    return is_collision

## p_obstacle : n*3 np.array

## r_obstacle : n*1 np.array

## all_po, all_radius 3D array



# to be complete

def detect_path_collision(S, all_initpos, all_radius, p_obstacle, r_obstacle, theta_a, theta_b):

    is_collision = 0

    for i in range(0,100,1):

        s = 0.01 * i

        theta_a=np.array(theta_a)

        theta_b=np.array(theta_b)

        theta_test = s * theta_b + (1-s) * theta_a

        test_position = find_all_target_position(S, theta_test, all_initpos)

        if check_current_collision(test_position,all_radius,p_obstacle,r_obstacle) == 1:


            is_collision = 1

            break

    return is_collision



def choose_closest_node(curr, qtheta, shortest, shortestdist):

    dist = sla.norm(np.array(curr.data)-np.array(qtheta))

    if shortest == None:

        shortest = curr

        shortestdist = dist

    elif (dist < shortestdist):

        shortest = curr

        shortestdist = dist



    if (curr.left == None and curr.right == None):

        return shortest

    if(curr.left != None and curr.right==None):

        return choose_closest_node(curr.left, qtheta, shortest, shortestdist)

    if (curr.right != None and curr.left == None):

        return choose_closest_node(curr.right, qtheta, shortest, shortestdist)

    if (curr.right!=None and curr.left!=None):

        l = choose_closest_node(curr.left, qtheta, shortest, shortestdist)

        r = choose_closest_node(curr.right, qtheta, shortest, shortestdist)

        distl = sla.norm(np.array(l.data)-np.array(qtheta))

        distr = sla.norm(np.array(r.data)-np.array(qtheta))

        if (distl < distr):

            return l

        return r



# to be complete

# return theta as a n*6 np array

def path_planning(S, all_initpos, all_radius, p_obstacle, r_obstacle, theta_start, theta_end):

    rootstart = Tree()

    theta_start = np.array(theta_start)

    rootstart.data = theta_start.reshape(6,1)

    rootend = Tree()

    theta_end = np.array(theta_end)

    rootend.data = theta_end.reshape(6,1)


    if (detect_path_collision(S, all_initpos, all_radius, p_obstacle, r_obstacle, rootstart.data.tolist(), rootend.data.tolist()) != 1):
        #print(1)
        thetaret = rootstart.data;
        thetaret = np.append(thetaret, rootend.data, axis=1)
        return thetaret

    while(1):

        # random a set of theta


        qrand = np.random.rand(6,1)*2*np.pi-np.pi

        qrand = qrand.reshape(6,1)



        # check if qrand in collision

        curr_q_pos = find_all_target_position(S, qrand, all_initpos)

        if (check_current_collision(curr_q_pos,all_radius,p_obstacle,r_obstacle) == 1):


            continue






        # to this point curr_q_pos should be in free space




        # select closest start node

        closest_start = choose_closest_node(rootstart, qrand, None, 1000000)





        rets = detect_path_collision(S, all_initpos, all_radius, p_obstacle, r_obstacle, closest_start.data.tolist(), qrand.tolist())

        if (rets == 0):

            # add to start tree
            #print(3)

            currT = Tree();

            currT.data = qrand

            currT.parent = closest_start

            if (closest_start.left == None):

                closest_start.left = currT

            else:

                closest_start.right = currT



        # select closest end node

        closest_end = choose_closest_node(rootend, qrand, None, 1000000)



        rete = detect_path_collision(S, all_initpos, all_radius,p_obstacle,r_obstacle, qrand.tolist(), closest_end.data.tolist())

        if (rets == 0 and rete==0):

            # calculate thetas and end

            thetas = qrand;

            curr = currT

            while (curr.parent != None):

                curr = curr.parent

                thetas = np.insert(thetas, [0], curr.data, axis=1)



            curre = closest_end

            while (curre.parent != None):

                thetas = np.append(thetas, curre.data, axis=1)

                curre = curre.parent

            thetas = np.append(thetas, curre.data, axis=1)



            return thetas



        if (rete == 0):

            #print(2)

            # add to end Tree

            currTe = Tree();

            currTe.data = qrand

            currTe.parent = closest_end

            if (closest_end == None):

                closest_end.left = currTe

            else:

                closest_end.right = currTe



    # should not reach here

    return None



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


############## Get all the dummy handles #########################################
result, Dummy0 = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
result, Dummy1 = vrep.simxGetObjectHandle(clientID, 'Dummy1', vrep.simx_opmode_blocking)
result, Dummy2 = vrep.simxGetObjectHandle(clientID, 'Dummy2', vrep.simx_opmode_blocking)
result, Dummy3 = vrep.simxGetObjectHandle(clientID, 'Dummy3', vrep.simx_opmode_blocking)
result, Dummy4 = vrep.simxGetObjectHandle(clientID, 'Dummy4', vrep.simx_opmode_blocking)
result, Dummy5 = vrep.simxGetObjectHandle(clientID, 'Dummy5', vrep.simx_opmode_blocking)
result, Dummy6 = vrep.simxGetObjectHandle(clientID, 'Dummy6', vrep.simx_opmode_blocking)
result, Dummy7 = vrep.simxGetObjectHandle(clientID, 'Dummy7', vrep.simx_opmode_blocking)
result, Dummy8 = vrep.simxGetObjectHandle(clientID, 'Dummy8', vrep.simx_opmode_blocking)
result, Dummy9 = vrep.simxGetObjectHandle(clientID, 'Dummy9', vrep.simx_opmode_blocking)
result, Dummy10 = vrep.simxGetObjectHandle(clientID, 'Dummy10', vrep.simx_opmode_blocking)
result, Dummy11 = vrep.simxGetObjectHandle(clientID, 'Dummy11', vrep.simx_opmode_blocking)

result, obstacle1 = vrep.simxGetObjectHandle(clientID, 'obstacle1', vrep.simx_opmode_blocking)
result, obstacle2 = vrep.simxGetObjectHandle(clientID, 'obstacle2', vrep.simx_opmode_blocking)
result, obstacle3 = vrep.simxGetObjectHandle(clientID, 'obstacle3', vrep.simx_opmode_blocking)
result, obstacle4 = vrep.simxGetObjectHandle(clientID, 'obstacle4', vrep.simx_opmode_blocking)
result, obstacle5 = vrep.simxGetObjectHandle(clientID, 'obstacle5', vrep.simx_opmode_blocking)
result, obstacle6 = vrep.simxGetObjectHandle(clientID, 'obstacle6', vrep.simx_opmode_blocking)
result, obstacle7 = vrep.simxGetObjectHandle(clientID, 'obstacle7', vrep.simx_opmode_blocking)
result, obstacle8 = vrep.simxGetObjectHandle(clientID, 'obstacle8', vrep.simx_opmode_blocking)
result, obstacle9 = vrep.simxGetObjectHandle(clientID, 'obstacle9', vrep.simx_opmode_blocking)
result, obstacle10 = vrep.simxGetObjectHandle(clientID, 'obstacle10', vrep.simx_opmode_blocking)

obstacleHandle = [obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10]

# Start simulation

vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait 0.5 seconds

time.sleep(0.5)

# set obstacles (default 3, add more, at most add 7)
user_input = input("please input number of obstacles to be added, any number from 0 to 7")
num_obstacle = int(user_input)

add_positions = []
for i in range(num_obstacle):
    user_input = input("please input coordinate of ball related to the first joint of UR3 (x,y,z) separated by ,")
    input_list = user_input.split(',')
    obstacle_co = [float(z) for z in input_list]
    #print(obstacle_co)
    add_positions.append(obstacle_co)

for i in range(0,2):

    for x in range(0,6):

        vrep.simxSetJointTargetPosition(clientID, jointHandles[x], 0, vrep.simx_opmode_oneshot)

    S = get_all_s ( jointHandles, clientID)

    user_input = input("please input 6 start angles separated by ,")

    input_list = user_input.split(',')

    theta_start = [float(z) for z in input_list]

    user_input = input("please input 6 start angles separated by ,")

    input_list = user_input.split(',')

    theta_end = [float(z) for z in input_list]

    #user_input = input("please input coordinate of ball related to the first joint of UR3 (x,y,z) separated by ,")

    #input_list = user_input.split(',')

    #obstacle_co = [float(z) for z in input_list]
    #print(obstacle_co)


    test_limit=0

    for j in range(0,5):

    	if theta_start[j]<-np.pi or theta_start[j]>2*np.pi or theta_end[j]<-np.pi or theta_end[j]>2*np.pi:

    		test_limit=1

    if test_limit == 1:

    	print('######################################')

    	print('##### Violating joint limit! #########')

    	print('######################################')

#################################    Get all the Dummy location!  ################################

    result, Dummy0_l = vrep.simxGetObjectPosition(clientID, Dummy0, jointHandles[0], vrep.simx_opmode_blocking)
    result, Dummy1_l = vrep.simxGetObjectPosition(clientID, Dummy1, jointHandles[0], vrep.simx_opmode_blocking)
    result, Dummy2_l = vrep.simxGetObjectPosition(clientID, Dummy2, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy3_l = vrep.simxGetObjectPosition(clientID, Dummy3, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy4_l = vrep.simxGetObjectPosition(clientID, Dummy4, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy5_l = vrep.simxGetObjectPosition(clientID, Dummy5, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy6_l = vrep.simxGetObjectPosition(clientID, Dummy6, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy7_l = vrep.simxGetObjectPosition(clientID, Dummy7, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy8_l = vrep.simxGetObjectPosition(clientID, Dummy8, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy9_l = vrep.simxGetObjectPosition(clientID, Dummy9, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy10_l = vrep.simxGetObjectPosition(clientID, Dummy10, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy11_l = vrep.simxGetObjectPosition(clientID, Dummy11, jointHandles[0],vrep.simx_opmode_blocking)

    all_initpos = [  [ Dummy0_l,Dummy1_l],[],[Dummy2_l,Dummy3_l,Dummy4_l],[Dummy5_l,Dummy6_l,Dummy7_l],[Dummy8_l,Dummy9_l],[Dummy10_l,Dummy11_l] ,[] ]
    all_radius =  [  [ [0.07], [0.05]  ],[],[ [0.055] , [0.035] ,  [0.025]],[ [0.04] ,[0.0325] , [0.0325]],[ [0.05]  , [0.0325]],[ [0.0325] , [0.0325] ] ,[] ]

    for i in range(len(add_positions)):
        returnCode=vrep.simxSetObjectPosition(clientID,obstacleHandle[i],jointHandles[0],add_positions[i],vrep.simx_opmode_blocking)

    result, obstacle1_l = vrep.simxGetObjectPosition(clientID, obstacle1, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle2_l = vrep.simxGetObjectPosition(clientID, obstacle2, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle3_l = vrep.simxGetObjectPosition(clientID, obstacle3, jointHandles[0], vrep.simx_opmode_blocking)

    p_obstacle = np.array([obstacle1_l,obstacle2_l,obstacle3_l])
    #print(p_obstacle)
    r_obstacle = np.array([[0.07],[0.07],[0.1]])

    for i in range(len(add_positions)):
        result, obstaclei_l = vrep.simxGetObjectPosition(clientID, obstacleHandle[i], jointHandles[0], vrep.simx_opmode_blocking)
        #print(obstaclei_l)
        p_obstacle =np.append(p_obstacle,[obstaclei_l],axis=0)
        r_obstacle = np.append(r_obstacle, [[0.03]], axis=0)







    theta_path = path_planning(S, all_initpos, all_radius, p_obstacle, r_obstacle, theta_start, theta_end)

    number_set = len(theta_path[0])




    for i in range(0,number_set):

        vrep.simxSetJointTargetPosition(clientID, jointHandles[0], theta_path[0][i], vrep.simx_opmode_oneshot)

        time.sleep(0.1)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[1], theta_path[1][i], vrep.simx_opmode_oneshot)

        time.sleep(0.1)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[2], theta_path[2][i], vrep.simx_opmode_oneshot)

        time.sleep(0.1)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[3], theta_path[3][i], vrep.simx_opmode_oneshot)

        time.sleep(0.1)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[4], theta_path[4][i], vrep.simx_opmode_oneshot)

        time.sleep(0.1)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[5], theta_path[5][i], vrep.simx_opmode_oneshot)

        time.sleep(0.4)

    theta_path = path_planning(S, all_initpos, all_radius, p_obstacle, r_obstacle, theta_end, [0,0,0,0,0,0])

    number_set = len(theta_path[0])




    for i in range(0,number_set):

        vrep.simxSetJointTargetPosition(clientID, jointHandles[0], theta_path[0][i], vrep.simx_opmode_oneshot)

        #time.sleep(0.5)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[1], theta_path[1][i], vrep.simx_opmode_oneshot)

       # time.sleep(0.5)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[2], theta_path[2][i], vrep.simx_opmode_oneshot)

        #time.sleep(0.5)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[3], theta_path[3][i], vrep.simx_opmode_oneshot)

        #time.sleep(0.5)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[4], theta_path[4][i], vrep.simx_opmode_oneshot)

        #time.sleep(0.5)

        vrep.simxSetJointTargetPosition(clientID, jointHandles[5], theta_path[5][i], vrep.simx_opmode_oneshot)

        time.sleep(1)





 #   err, contact = vrep.simxGetCollisionHandle (clientID, 'Collision2' ,vrep.simx_opmode_blocking)

 #   err, collision = vrep.simxReadCollision (clientID, contact , vrep.simx_opmode_streaming)



    #return_code,state_self=vrep.simxGetIntegerSignal(clientID, "co_self", vrep.simx_opmode_blocking)

    #return_code,state_ground =vrep.simxGetIntegerSignal(clientID, "co_ground", vrep.simx_opmode_blocking)

    #return_code,state_box =vrep.simxGetIntegerSignal(clientID, "co_box", vrep.simx_opmode_blocking)

    #time.sleep(0.5)



    #if state_self:

    	#print('######################################')

    	#print('#### Collide with robot itself! ######')

    	#print('######################################')



    #if state_ground:

    	#print('######################################')

    	#print('###### Collide with the ground! ######')

    	#print('######################################')



    #if state_box:

    	#print('######################################')

    	#print('###### Collide with the cylinder! ####')

    	#print('######################################')

    #if state_self == 0 and state_ground == 0 and state_box == 0:

    	#print('######################################')

    	#print('############ No Collision~ ###########')

    	#print('######################################')



vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)





# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):

vrep.simxGetPingTime(clientID)

# Close the connection to V-REP

vrep.simxFinish(clientID)
