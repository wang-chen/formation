#!/usr/bin/env python
''' The state equation, dx/dt = f(x,t0,u)
    Computes the derivative of state at time t0 on the condition of input u.
    x[0:3] --> Position in ned frame
    x[3:7] --> Quaternion
    x[7:10] --> Velocity in ned frame
    x[10]   --> Bais in Yaw direction of body frame
    
    u[0:3] --> Accelaration in body frame
    u[3:6] --> Angle rate of body frame expressed in inertial frame  '''

from sslib import *

g  = -9.80665
Q  = zeros((11,11))
Q[ 0:3,  0:3] =  0.5*eye(3)#*10
Q[ 3:7,  3:7] =  0.01*eye(4)#*10
Q[7:9, 7:9] =  50*eye(2)#/2
Q[9,9] =  400#/2
Q[10,10]      =  0.000000001

K  = array([[320, 0,   320],
            [  0, 240, 240]])


uwbanchor = array([[-2,-2,0.2],[-2,2,1.2],[2,2,1.3],[2,-2,1.8]])
visionanchor = array([[-2, -3,0 ],[-1, -3, 0],[1, -3, 0],[2, -3, 0]])

def state_equation(x, t0, u):
    [p, q, v, b]     = [x[0:3], x[3:7], x[7:10], x[10]]
    [a, wx, wy, wz]  = [u[0:3], u[3], u[4], u[5]]        
    [qx, qy, qz, qw] = [q[0], q[1], q[2], q[3]]  
    #[qw, qx, qy, qz] = [q[0], q[1], q[2], q[3]] 
    #positon transition
    dev_p = v
    #quaternion transition
    #===========================================================================
    #R =  array([[0,  -wx, -wy, -wz ],
    #            [wx,   0,  wz, -wy ],
    #            [wy, -wz,  0 ,  wx ],
    #            [wz,  wy, -wx,  0  ]])
    #===========================================================================
    R = array([[0,   wz,  -wy,  wx ],
               [-wz,   0,  wx,  wy ],
               [ wy, -wx,  0 ,  wz ],
               [-wx, -wy,  -wz,  0  ]])
    dev_q = 0.5 * dot(R, q)   
    #veloctiy transition
    T = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
               [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
               [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])
    dev_v = dot(T, a) + array([0,0,g])   
    #yaw bias transition
    dev_b = 0  
    #merge state transition
    dev_x = hstack((dev_p, dev_q, dev_v, dev_b))
    return dev_x

def transition_function(xk, uk, del_t):
    T  = del_t
    [pk, qk, vk, bk] = [xk[0:3], xk[3:7], xk[7:10], xk[10]]
    pvk = hstack((pk,vk))
    [a, wx, wy, wz]  = [uk[0:3], uk[3], uk[4], uk[5]]        
    [qx, qy, qz, qw] = [qk[0], qk[1], qk[2], qk[3]]
    qk = array([qw, qx, qy, qz])
    u = hstack((a,1))
    #quaternion transition
    w = linalg.norm(array((wx,wy,wz)))
    if w==0:
        r0, r1, r2, r3 = 1, 0, 0, 0
    else:
        r0, r1, r2, r3 = cos(T*w/2), sin(T*w/2)*wx/w, sin(T*w/2)*wy/w, sin(T*w/2)*wz/w
    Aq = array([[r0, -r1, -r2, -r3],
                [r1,  r0,  r3, -r2],
                [r2, -r3,  r0,  r1],
                [r3,  r2, -r1,  r0]])
    qk1 = dot(Aq,qk)
    qk1 = qk1/linalg.norm(qk1)
    #position and velocity transition
    Apv = array([[1, 0, 0, T, 0, 0],
                 [0, 1, 0, 0, T, 0],
                 [0, 0, 1, 0, 0, T],
                 [0, 0, 0, 1, 0 ,0],
                 [0, 0, 0, 0, 1 ,0],
                 [0, 0, 0, 0, 0 ,1]])
    Btt = array([[T, 0, 0, T*T/2,   0,     0  ],
                 [0, T, 0,   0,   T*T/2,   0  ],
                 [0, 0, T,   0,    0,    T*T/2],
                 [0, 0, 0,   T,     0 ,    0  ],
                 [0, 0, 0,   0,     T ,    0  ],
                 [0, 0, 0,   0,     0 ,    T  ]])
    Tned = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
                  [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])
    Bpv = dot(Btt, vstack((zeros((3,4)),column_stack((Tned ,array([0,0,g]))))))
    pvk1 = dot(Apv,pvk) + dot(Bpv,u)
    #bias transition
    bk1 = bk
    #merge state transition
    return hstack((pvk1[0:3], array((qk1[1],qk1[2],qk1[3],qk1[0])), pvk1[3:6], bk1))

class VisionlLocation:
    def __init__(self, delt_time, camera_matrix):
        self.N = 11
        self.M = 12
        self.x = zeros((1,self.N))[0]
        self.R = zeros((12,12))
        self.R[0:8,0:8] = eye(8)*10
        self.R[8:12,8:12] = eye(4)*0.0001
        self.state_equation = copy.deepcopy(state_equation)
        self.u = tuple([[0,0,-g,0,0,0]])
        self.delt_time = delt_time
        self.Q = copy.deepcopy(Q*delt_time)
        self.K = camera_matrix
        self.ukfinit()
        
    def setQ(self, Q):
        self.Q = Q * self.delt_time
        self.ukfinit()
 
    def ukfinit(self):
        self.ukf = AdditiveUnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = self.Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = self.Q)

    def locate(self, state, state_cov, delt_time, image_points, anchor_pos, quaternion, linear_acc, angular_rate):
        self.anchor_pos = anchor_pos
        self.delt_time  = delt_time 
        self.u = tuple([hstack((linear_acc, angular_rate))])
        (self.x, self.P) = self.ukf.filter_update(state, state_cov*delt_time, hstack((image_points, quaternion)))
        return (self.x, self.P)

    def transition_function(self, state):
        #return odeint(self.state_equation, state, [0, self.delt_time], self.u)[1]
        return transition_function(state, self.u[0], self.delt_time)

    def observation_function(self, state):
        #return hstack((linalg.norm(state[0:3] - self.anchor_pos), state[3:7]))
        return hstack((array([ dot(self.K, state[0:3]-self.anchor_pos[i]) for i in xrange(4)]).reshape((8)), state[3:7]))


class UWBLocation:
    def __init__(self, delt_time):
        self.N = 11
        self.M = 5
        self.x = zeros((1,self.N))[0]
        self.R = zeros((5,5))
        self.R[0,0] = 0.1
        self.R[1:5,1:5] = eye(4)*0.0001
        self.state_equation = copy.deepcopy(state_equation)
        self.u = tuple([[0,0,-g,0,0,0]])
        self.delt_time = delt_time
        self.Q = copy.deepcopy(Q*delt_time)
        self.ukfinit()
        
    def setQ(self, Q):
        self.Q = Q * self.delt_time
        self.ukfinit()

              
    def ukfinit(self):
        self.ukf = AdditiveUnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = self.Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = self.Q)

    def locate(self, state, state_cov, delt_time, anchor_dis, anchor_pos, quaternion, linear_acc, angular_rate):
        self.anchor_pos = anchor_pos
        self.delt_time  = delt_time 
        self.u = tuple([hstack((linear_acc, angular_rate))])
        (self.x, self.P) = self.ukf.filter_update(state, state_cov*delt_time, hstack((anchor_dis, quaternion)))
        return (self.x, self.P)

    def transition_function(self, state):
        #return odeint(self.state_equation, state, [0, self.delt_time], self.u)[1]
        return transition_function(state, self.u[0], self.delt_time)

    def observation_function(self, state):
        return hstack((linalg.norm(state[0:3] - self.anchor_pos), state[3:7]))

