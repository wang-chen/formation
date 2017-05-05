#!/usr/bin/env python
from matplotlib.lines import lineStyles
from numpy.oldnumeric.linear_algebra import inverse
from pykalman import UnscentedKalmanFilter, AdditiveUnscentedKalmanFilter
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from mpl_toolkits.mplot3d import Axes3D
from numpy import *
import time
import copy

g = -9.80665   
Q  = zeros((10,10))
Q[0:3, 0:3] =  0.0001*eye(3)
Q[3:6, 3:6] =  0.5*eye(3)
Q[6:9, 6:9] =  0.0064*eye(3)
#Q[8,8]      =  0.0064
Q[9,9]      =  0.00000000001



def state_equation(x, t0, u):
    ''' Defined by Jeffsan Wang
        The state equation, dx/dt = f(x,t0,u)
        Computes the derivative of state at time t0 on the condition of input u.
        x[0:3] --> Position in ned frame
        x[3:6] --> Euler angle of body frame expressed in inertial frame
        x[6:9] --> Velocity in aircraft body frame
        x[9]   --> Bais in Yaw direction of body frame
        
        u[0:3] --> Accelaration in body frame
        u[3:6] --> Angle rate of body frame expressed in inertial frame  '''

    [pos, eul, vel, bias]    = [x[0:3], x[3:6], x[6:9], x[9]]
    [ax, ay, az, wx, wy, wz] = [u[0], u[1], u[2], u[3], u[4], u[5]]        
    [phi, theta, psi]        = [eul[0], eul[1], eul[2]]
    [vx, vy, vz]             = [vel[0], vel[1], vel[2]]
    
    #positon transition
    [cp, sp, ct, st, cs, ss] = [cos(phi), sin(phi), cos(theta), 
                                sin(theta), cos(psi), sin(psi)]   
    T = array([[        ct*cs,          ct*ss,        -st ],
               [sp*st*cs - cp*ss, sp*st*ss + cp*cs,  sp*ct],
               [cp*st*cs + sp*ss, cp*st*ss - sp*cs,  cp*ct]])
    dev_pos   = dot(inverse(T), vel)

    
    #euler angle transition
    tt = tan(theta)
    R = array([[1, sp * tt, cp * tt ],
               [0,    cp,     -sp   ],
               [0, sp / ct, cp / ct ]])
    #print "R",R
    dev_euler = dot(R, [wx, wy, wz])
    #print 'dev_euler', dev_euler
    
    #veloctiy transition
    dev_vx = ax - g * st      - wy * vz + wz * vy
    dev_vy = ay + g * ct * sp - wz * vx + wx * vz
    dev_vz = az + g * ct * cp - wx * vy + wy * vx
    dev_vel = [dev_vx, dev_vy, dev_vz]
    
    #yaw bias transition
    dev_bias = 0
    
    #merge state transition
    dev_x = hstack((dev_pos, dev_euler, dev_vel, dev_bias))
    #print 'dev_x ', dev_x
    return dev_x

class UWBLocation:
    def __init__(self):
        self.N = 10
        self.M = 4
        self.x = zeros((1,self.N))[0]
        self.R = eye(4)*0.0001
        self.R[1:4,1:4] = eye(3)*0.000001
        self.P = Q
        self.time = -1
        self.ukfinit()
        self.state_equation = copy.deepcopy(state_equation)
        self.u = tuple([[0,0,-g,0,0,0]])
              
    def ukfinit(self):

        self.ukf = AdditiveUnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = Q)

    def locate(self, state, state_cov, delt_time, anchor_dis, anchor_pos, euler_angle, linear_acc, angular_rate):
        self.anchor_pos = anchor_pos
        self.delt_time  = delt_time 
        # [phi, theta, psi]        = [state[3], state[4], state[5]]
        # [cp, sp, ct, st, cs, ss] = [cos(phi), sin(phi), cos(theta), 
        #                             sin(theta), cos(psi), sin(psi)]   

        #self.u = tuple([[g * st, -g * ct * sp, -g * ct * cp,0,0,0]])
        self.u = tuple([hstack((linear_acc, angular_rate))])
        #print self.u

        (self.x, self.P) = self.ukf.filter_update(state, state_cov, hstack((anchor_dis, euler_angle)))
        return (self.x, self.P)

    def transition_function(self, state):
        return odeint(self.state_equation, state, [0, self.delt_time], self.u)[1]

    def observation_function(self, state):
        return hstack((linalg.norm(state[0:3] - self.anchor_pos), state[3:6]))









class FastVisionLocation:
    def __init__(self):
        self.M = 2
        self.N = 3
        self.A = [[1, 0, 0], [0, 1, 0],[0, 0, 1]]
        self.C = [[1, 0, 0], [0, 1, 0]]
        kf = KalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                          transition_matrices=self.A, observation_matrices=self.C)

class IMULocation:
    def __init__(self):
        self.N = 10
        self.M = 3
        self.x = zeros((1,self.N))[0]
        self.R = eye(3)*0.02
        self.P = Q
        self.time = -1
        self.ukfinit()
        self.state_equation = copy.deepcopy(state_equation)
        
    def ukfinit(self):

        self.ukf = AdditiveUnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = Q)

    def locate(self, state, state_cov, delt_time, euler_angle, linear_acc, angular_rate):
        self.delt_time = delt_time
        self.u = tuple((hstack((linear_acc, angular_rate))))
        (self.x, self.P) = self.ukf.filter_update(state, state_cov, euler_angle)
        print linalg.norm(euler_angle-self.x[3:6])
        return (self.x, self.P)

    def transition_function(self, state):
        return odeint(self.state_equation, state, [0, self.delt_time], tuple([self.u]))[1]

    def observation_function(self, state):
        return hstack((state[3:6]))