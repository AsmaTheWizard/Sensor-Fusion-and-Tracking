# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

    
class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass


    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        matrix_ = np.eye(params.dim_state)
        matrix_[0,3] = params.dt
        matrix_[1,4] = params.dt
        matrix_[2,5] = params.dt
        return np.matrix(matrix_)
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        Q_matrix = np.zeros((params.dim_state, params.dim_state))
        Q_matrix[0, 0] = Q_matrix[1, 1] = Q_matrix[2, 2] = (params.dt ** 4) / 4
        Q_matrix[0, 3] = Q_matrix[1, 4] = Q_matrix[2, 5] = (params.dt ** 3) / 2
        Q_matrix[3, 0] = Q_matrix[4, 1] = Q_matrix[5, 2] = (params.dt ** 3) / 2
        Q_matrix[3, 3] = Q_matrix[4, 4] = Q_matrix[5, 5] = (params.dt ** 2)
        Q_matrix = Q_matrix * params.q
        return np.matrix(Q_matrix)
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F = self.F()
        F_t = F.transpose()
        Q = self.Q()
        x = F * track.x
        P = F * track.P * F_t + Q
        track.set_x(x)
        track.set_P(P)
        
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, meas.sensor.get_H(track.x))
        K = track.P * H.transpose() * S.I
        x = track.x + (K * gamma)
        I = np.eye(params.dim_state)
        P = (I - K * H) * track.P
        track.set_P(P)
        track.set_x(x)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        return meas.z - meas.sensor.get_hx(track.x)
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return H * track.P * H.transpose() + meas.R
        
        ############
        # END student code
        ############ 