import cv2
import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.common import Q_discrete_white_noise


class KalmanFilter(object):
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """

        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """

        #define sampling rate
        self.dt = dt

        #define control input variables
        self.u = np.array([[u_x],[u_y]])

        #initial state
        self.x = np.array([[0],[0],[0],[0]])

        #define the state transition matrix A
        self.A = np.array([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        #define the Control Input Matrix B
        self.B = np.array([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt,0],
                            [0,self.dt]])

        #define measurement mapping matrix
        self.H = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        #Initial Process Noise Covariance
        self.Q = np.array([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

        #Initial Measurement Noise Covariance

        self.R = np.array([[x_std_meas**2,0],
                           [0, y_std_meas**2]])

        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

    def predict(self):

        # update time state
        # x_k = Ax_ (k-1) + Bu_(k-1)
        self.x = np.dot(self.A, self.x) + np.dot (self.B, self.u)

        #calculate error covariance

        #P = A*P*A' + Q
        self.P = np.dot(np.dot(self.A, self.P),self.A.T) + self.Q

        return self.x[0:2]


    def update(self,z):

        # made up variable for future inversion S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))

        #identity matrix for future calculations
        I = np.eye(self.H.shape[1])


        # Update error covariance matrix
        #self.P = (I - (K * self.H)) * self.P
        self.P = np.dot((I - np.dot(K , self.H)), self.P)

        return self.x[0:2]


class raketa:

    def __init__(self, name):

        self.name = name

    def get_name(self):

        return self.name



    def set_coordinates (self, x_coordinate,y_coordinate):
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate

        return print(f" Object's name - {self.name}, its  x coordinate {self.x_coordinate} and y coordinate {self.y_coordinate} ")

    def set_identification_nr (self, identification_nr):
        self.identification_nr= identification_nr

    def get_coordinates(self):
        return self.x_coordinate, self.y_coordinate





















