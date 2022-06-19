#!/usr/bin/env python3

import queue
from geometry_msgs.msg import Twist

################################################################################

class PIDController:
    '''
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    '''
    def __init__(self, kP, kI, kD, kS):
        self.kP       = kP # Proportional gain
        self.kI       = kI # Integral gain
        self.kD       = kD # Derivative gain
        self.kS       = kS # Saturation constant (error history buffer size)
        self.err_int  = 0 # Error integral
        self.err_dif  = 0 # Error difference
        self.err_prev = 0 # Previous error
        self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
        self.t_prev   = 0 # Previous time

    def control(self, err, t):
        '''
        Generate PID controller output.

        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp

        :return u: PID controller output
        '''
        dt = t - self.t_prev # Timestep
        if dt > 0.0:
            self.err_hist.put(err)
            self.err_int += err
            if self.err_hist.full(): # Jacketing logic to prevent integral windup
                self.err_int -= self.err_hist.get() # Rolling FIFO buffer
            self.err_dif = (err - self.err_prev)
            u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt)
            self.err_prev = err
            self.t_prev = t
            return u # Control signal

################################################################################

class PolarController:
    '''
    Generates control action proportional to the errors in distance to goal (rho),
    start heading error (alpha) and goal heading error (beta).
    '''
    def __init__(self, k_rho, k_alpha, k_beta, max_lin_vel, max_ang_vel):
        self.k_rho       = k_rho # Proportional gain for rho
        self.k_alpha     = k_alpha # Proportional gain for alpha
        self.k_beta      = k_beta # Proportional gain for beta
        self.max_lin_vel = max_lin_vel # Maximum allowed linear velocity in m/s
        self.max_ang_vel = max_ang_vel # Maximum allowed angular velocity in rad/s

    def control(self, rho, alpha, beta):
        '''
        Generate polar controller output.

        :param rho  : Length of segment (distance) joining current position to goal in meters
        :param alpha: Angle of the segment w.r.t current heading in radians
        :param beta : Goal heading w.r.t segment extension in radians

        :return u: Polar controller output (ROS Twist message)
        '''
        u = Twist()
        u.linear.x = min(self.k_rho * rho, self.max_lin_vel)
        u.angular.z = min(self.k_alpha * alpha + self.k_beta * beta, self.max_ang_vel)
        return u
