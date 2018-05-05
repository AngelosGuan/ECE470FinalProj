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

def ad(arr):
    R=arr[0:3,0:3]
    p=arr[0:3,3]
    A=np.kron(np.eye(2),R)
    B=np.kron(np.array([[0,0],[1,0]]),skew3(p).dot(R))
    mat=A+B
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

def skew3(arr):
    a=arr[0]
    b=arr[1]
    c=arr[2]
    mat = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
    return mat

# return value: obj_flag, x_cen, y_cen
# obj_flag is a integer representing the color of the deteceted object, 1 for red, 2 for green, 3 for indigoid
# x_cen, y_cen is the center of the object, represented by pixels
def get_object_type_and_center(resolution, ig):
    obj_flag=0
    object_array_x = [0,0,0,0]
    object_array_y = [0,0,0,0]
    object_array_count = [0,0,0,0]
    for i in range(4):
        object_array_x[i] = 0
        object_array_y[i] = 0
        object_array_count[i] = 0
    # obj_flag: red:1, green:2, indigoid: 3
    for i in range(0,resolution[0]):
        for j in range(0,resolution[1]):
            if(sla.norm(ig[i][j])!=0):
                rgb = np.array(ig[i][j])
                if (rgb[0]>200 and rgb[1]<10 and rgb[2] < 10):
                    obj_flag = 1
                    object_array_x[1]+=i
                    object_array_y[1]+=j
                    object_array_count[1]+=1
                elif (rgb[0]<10 and rgb[1]>200 and rgb[2]>200):
                    obj_flag = 3
                    object_array_x[3]+=i
                    object_array_y[3]+=j
                    object_array_count[3]+=1
                elif (rgb[0]<10 and rgb[1]>200 and rgb[2]<10):
                    obj_flag = 2
                    object_array_x[2]+=i
                    object_array_y[2]+=j
                    object_array_count[2]+=1
    if (obj_flag!=0):
        x_cen = object_array_x[obj_flag]/object_array_count[obj_flag]
        y_cen = object_array_y[obj_flag]/object_array_count[obj_flag]
    else:
        x_cen = None
        y_cen = None
        obj_flag = None
    return obj_flag, x_cen, y_cen

def inverse_k(s,M,M_destination,clientID):
    #theta=[]
    flag=0
    V=[]
    for i in range(6):
        V.append([2])    # random V value that doesn't satisfy the result
    #theta=np.array(np.random.rand(6,1))
    theta=np.array([[0.5],[0.5],[0.5],[0.5],[0.5],[0.5]])
    V=np.array(V)
    error=0.1
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
        if (np.absolute(sla.norm(V)-sla.norm(V_former)) < 1e-14 ):
            flag=1
            print('Can not reach the point.')
            return flag,theta
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

def get_M(joints, gripper,clientID):
    s = []
    a = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[0,0,1],[0,1,0]]);
    x = 0
    y = 0
    z = 0
    vector = [[0],[0],[0]]
    # for i in range(0,6):
    # 	result,vector=vrep.simxGetObjectPosition(clientID,joints[i],joints[0],vrep.simx_opmode_blocking)
    # 	if result != vrep.simx_return_ok:
    # 		raise Exception('could not get position')
    # 	q=np.array([[vector[0]],[vector[1]],[vector[2]]])
    # 	s.append(getS(a[i],q)[:,0])
    # s=np.array(s).T
    result,vector=vrep.simxGetObjectPosition(clientID,gripper,joints[0],vrep.simx_opmode_blocking)
    result,theta_come =vrep.simxGetObjectQuaternion(clientID,gripper,joints[0],vrep.simx_opmode_blocking)
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
    return M

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
            print("Collision!")
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
        qrand = np.random.rand(6,1)*3*np.pi-np.array([[np.pi],[np.pi],[np.pi],[np.pi],[np.pi],[np.pi]])
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
# result, joint_end = vrep.simxGetObjectHandle(clientID, 'UR3_connection', vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#     raise Exception('could not get object handle for sixth joint')
jointHandles = []
jointHandles.append(joint_one_handle)
jointHandles.append(joint_two_handle)
jointHandles.append(joint_three_handle)
jointHandles.append(joint_four_handle)
jointHandles.append(joint_five_handle)
jointHandles.append(joint_six_handle)
result, gripper = vrep.simxGetObjectHandle(clientID, 'BarrettHand', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for the gripper')
result, visionss = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for the vision sensor')
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
result, Dummy12 = vrep.simxGetObjectHandle(clientID, 'Dummy12', vrep.simx_opmode_blocking)
result, Dummy13 = vrep.simxGetObjectHandle(clientID, 'Dummy13', vrep.simx_opmode_blocking)
result, Dummy14 = vrep.simxGetObjectHandle(clientID, 'Dummy14', vrep.simx_opmode_blocking)
result, obstacle1 = vrep.simxGetObjectHandle(clientID, 'obstacle1', vrep.simx_opmode_blocking)
result, obstacle2 = vrep.simxGetObjectHandle(clientID, 'obstacle2', vrep.simx_opmode_blocking)
result, obstacle3 = vrep.simxGetObjectHandle(clientID, 'obstacle3', vrep.simx_opmode_blocking)
result, obstacle4 = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sphere')
result, obstacle5 = vrep.simxGetObjectHandle(clientID, 'Sphere0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sphere')
result, obstacle6 = vrep.simxGetObjectHandle(clientID, 'Sphere1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sphere')
result, obstacle7 = vrep.simxGetObjectHandle(clientID, 'obstacle7', vrep.simx_opmode_blocking)
result, obstacle8 = vrep.simxGetObjectHandle(clientID, 'obstacle8', vrep.simx_opmode_blocking)
result, obstacle9 = vrep.simxGetObjectHandle(clientID, 'obstacle9', vrep.simx_opmode_blocking)
result, obstacle10 = vrep.simxGetObjectHandle(clientID, 'obstacle10', vrep.simx_opmode_blocking)
result, red_handle = vrep.simxGetObjectHandle(clientID, 'red_des', vrep.simx_opmode_blocking)
result, blue_handle = vrep.simxGetObjectHandle(clientID, 'blue_des', vrep.simx_opmode_blocking)
result, green_handle = vrep.simxGetObjectHandle(clientID, 'green_des', vrep.simx_opmode_blocking)
result,gripper_final=vrep.simxGetObjectPosition(clientID,red_handle,jointHandles[0],vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Vision_sensor position')
red_final_pose= np.array([[-1,0,0, gripper_final[0]],
[0,1,0, gripper_final[1]],
[0,0,-1, gripper_final[2]],
[0,0,0,1]])
result,gripper_final=vrep.simxGetObjectPosition(clientID,blue_handle,jointHandles[0],vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Vision_sensor position')
blue_final_pose= np.array([[-1,0,0, gripper_final[0]],
[0,1,0, gripper_final[1]],
[0,0,-1, gripper_final[2]],
[0,0,0,1]])
result,gripper_final=vrep.simxGetObjectPosition(clientID,green_handle,jointHandles[0],vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Vision_sensor position')
green_final_pose= np.array([[-1,0,0, gripper_final[0]],
[0,1,0, gripper_final[1]],
[0,0,-1, gripper_final[2]],
[0,0,0,1]])
obstacleHandle = [obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10]
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

for i in range(0,2):
    result=vrep.simxSetIntegerSignal(clientID,"start_state",1, vrep.simx_opmode_oneshot)
    result=vrep.simxSetIntegerSignal(clientID,"split_state",1, vrep.simx_opmode_oneshot)
    result=vrep.simxSetIntegerSignal(clientID,"belt_state",1, vrep.simx_opmode_oneshot)
    for x in range(0,6):
        vrep.simxSetJointTargetPosition(clientID, jointHandles[x], 0, vrep.simx_opmode_oneshot)
    S = get_all_s ( jointHandles, clientID)
    M = get_M(jointHandles, gripper,clientID)
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
    result, Dummy12_l = vrep.simxGetObjectPosition(clientID, Dummy12, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy13_l = vrep.simxGetObjectPosition(clientID, Dummy13, jointHandles[0],vrep.simx_opmode_blocking)
    result, Dummy14_l = vrep.simxGetObjectPosition(clientID, Dummy14, jointHandles[0],vrep.simx_opmode_blocking)
    all_initpos = [  [ Dummy0_l,Dummy1_l],[],[Dummy2_l,Dummy3_l,Dummy4_l],[Dummy5_l,Dummy6_l,Dummy7_l],[Dummy8_l,Dummy9_l],[Dummy10_l,Dummy11_l,Dummy12_l,Dummy13_l,Dummy14_l] ,[] ]
    all_radius =  [  [ [0.07], [0.05]  ],[],[ [0.055] , [0.035] ,  [0.025]],[ [0.04] ,[0.0325] , [0.0325]],[ [0.05]  , [0.0325]],[ [0.0325] , [0.0325],[0.0325],[0.0325],[0.044]] ,[] ]
    result, obstacle1_l = vrep.simxGetObjectPosition(clientID, obstacle1, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle2_l = vrep.simxGetObjectPosition(clientID, obstacle2, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle3_l = vrep.simxGetObjectPosition(clientID, obstacle3, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle4_l = vrep.simxGetObjectPosition(clientID, obstacle4, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle5_l = vrep.simxGetObjectPosition(clientID, obstacle5, jointHandles[0], vrep.simx_opmode_blocking)
    result, obstacle6_l = vrep.simxGetObjectPosition(clientID, obstacle6, jointHandles[0], vrep.simx_opmode_blocking)
    p_obstacle = np.array([obstacle1_l,obstacle2_l,obstacle3_l,obstacle4_l,obstacle5_l,obstacle6_l])

    r_obstacle = np.array([[0.07],[0.07],[0.1],[0.05],[0.05],[0.05]])

    while 1 :
        result=vrep.simxSetIntegerSignal(clientID,"belt_state",1, vrep.simx_opmode_oneshot)
        color_flag = None
        while color_flag == None:
            ret, resolution, ig = vrep.simxGetVisionSensorImage(clientID, visionss, 0, vrep.simx_opmode_streaming)
            while (vrep.simxGetConnectionId(clientID)!=-1):
                ret, resolution, ig = vrep.simxGetVisionSensorImage(clientID, visionss, 0, vrep.simx_opmode_buffer)
                if(ret==vrep.simx_return_ok):
                    ig = np.array(ig, dtype=np.uint8)
                    ig.resize([resolution[1],resolution[0],3])
                    color_flag,pixel_row,pixel_colume=get_object_type_and_center(resolution, ig)
                    resolution = resolution[0]
                    if color_flag!=None:
                        if  pixel_colume>resolution/2-10 and pixel_colume<resolution/2+10:
                            result=vrep.simxSetIntegerSignal(clientID,"belt_state",0, vrep.simx_opmode_oneshot)
                            break
                elif ret==vrep.simx_return_novalue_flag:
                    pass
        #pixel_row = 64
        #pixel_colume = 64           ############ get pixel coordinate from the function
        ################
        ############# convert it in to des_co
        ###########################
        #resolution = 128
        ratio = 0.4/resolution
        result,gripper_final=vrep.simxGetObjectPosition(clientID,visionss,jointHandles[0],vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get Vision_sensor position')
        gripper_final[0] = gripper_final[0]+(pixel_row-resolution/2)*ratio
        gripper_final[1] = gripper_final[1]+(pixel_colume-resolution/2)*ratio
        gripper_final[2] = 0.16   ############## May need modify ###################
        print(gripper_final)
        ############# convert des_co to a set of joint variables-inverse kinematic
        M_destination= np.array([[-1,0,0, gripper_final[0]],
        [0,1,0, gripper_final[1]],
        [0,0,-1, gripper_final[2]],
        [0,0,0,1]])
        flag,theta_end=inverse_k(S,M,M_destination,clientID)
        if flag ==1:
            print('error_theta!!!')
        flag,red_end=inverse_k(S,M,red_final_pose,clientID)
        if flag ==1:
            print('error_red!!!')
        flag,green_end=inverse_k(S,M,green_final_pose,clientID)
        if flag ==1:
            print('error_green!!!')
        flag,blue_end=inverse_k(S,M,blue_final_pose,clientID)
        if flag ==1:
            print('error_blue!!!')
        theta_start=[0,0,0,0,0,0]
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
        #for i in range(len(add_positions)):
        #    returnCode=vrep.simxSetObjectPosition(clientID,obstacleHandle[i],jointHandles[0],add_positions[i],vrep.simx_opmode_blocking)
        # for i in range(len(add_positions)):
        #     result, obstaclei_l = vrep.simxGetObjectPosition(clientID, obstacleHandle[i], jointHandles[0], vrep.simx_opmode_blocking)
        #     #print(obstaclei_l)
        #     p_obstacle =np.append(p_obstacle,[obstaclei_l],axis=0)
        #     r_obstacle = np.append(r_obstacle, [[0.03]], axis=0)
        theta_path = path_planning(S, all_initpos, all_radius, p_obstacle, r_obstacle, theta_start, theta_end)
        number_set = len(theta_path[0])
        for i in range(0,number_set):
            vrep.simxSetJointTargetPosition(clientID, jointHandles[0], theta_path[0][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[1], theta_path[1][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[2], theta_path[2][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[3], theta_path[3][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[4], theta_path[4][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[5], theta_path[5][i], vrep.simx_opmode_oneshot)
            time.sleep(1)
        time.sleep(4)
        result=vrep.simxSetIntegerSignal(clientID,"start_state",2, vrep.simx_opmode_oneshot)
        result=vrep.simxSetIntegerSignal(clientID,"split_state",2, vrep.simx_opmode_oneshot)
        time.sleep(5)
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
        if color_flag==1:
            sort_end=red_end
        elif color_flag==2:
            sort_end=green_end
        elif color_flag==3:
            sort_end=blue_end
        theta_path = path_planning(S, all_initpos, all_radius, p_obstacle, r_obstacle, [0,0,0,0,0,0], sort_end)
        number_set = len(theta_path[0])
        for i in range(0,number_set):
            vrep.simxSetJointTargetPosition(clientID, jointHandles[0], theta_path[0][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[1], theta_path[1][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[2], theta_path[2][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[3], theta_path[3][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[4], theta_path[4][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[5], theta_path[5][i], vrep.simx_opmode_oneshot)
            time.sleep(5)
        time.sleep(4)
        result=vrep.simxSetIntegerSignal(clientID,"start_state",1, vrep.simx_opmode_oneshot)
        time.sleep(3)
        result=vrep.simxSetIntegerSignal(clientID,"split_state",1, vrep.simx_opmode_oneshot)
        theta_path = path_planning(S, all_initpos, all_radius, p_obstacle, r_obstacle, sort_end,[0,0,0,0,0,0])
        number_set = len(theta_path[0])
        for i in range(0,number_set):
            vrep.simxSetJointTargetPosition(clientID, jointHandles[0], theta_path[0][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[1], theta_path[1][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[2], theta_path[2][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[3], theta_path[3][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[4], theta_path[4][i], vrep.simx_opmode_oneshot)
            #time.sleep(0.1)
            vrep.simxSetJointTargetPosition(clientID, jointHandles[5], theta_path[5][i], vrep.simx_opmode_oneshot)
            time.sleep(1)
        time.sleep(4)
        
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
