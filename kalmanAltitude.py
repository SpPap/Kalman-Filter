"""
This function intends to be used for Kalman Filter design for quadcopter altitude.
We use standard Kalman Filter with a Linear System Model.
This Kalman Filter will be used to take the very noisy barometer data in conjuction
with the accelerometeer data (derive z, z' from z'') to predict the system's state and predict height
with high accuracy.

   x = [position veloctity].T


Sensors used: - Barometric Pressure Sensor (MS5611)
              - Accelerometer (MPU9250 9DOF IMU)
              
              
INPUTS:  h_baro, acc_z, P_PredPrev
- h_baro:  raw height derived directly from MS5611 
           through conversion (Pressure [mbar] --> Height [m])
- acc_z:   raw acceleration in z-axis derived 
           from MPU9250's accelerometer (measured in g-force)
- P_PredPrev: Predicted (a priori) estimate covariance


OUTPUTS: x_Kalman, P_NextEst 
- x_Kalman:  Updated (a posteriori) state estimate [z-axis quad position, z-axis quad velocity] == [z, z']
- P_NextEst: Updated (a posteriori) estimate covariance
Author: Spiros Papadopoulos

"""
from scipy.linalg import inv
import numpy as np

###########################
# Kalman Filter Algorithm #
###########################

def KalmanAltitude2D(h_baro, acc_z, x_EstPrev, P_EstPrev, Ts):
   # Define sensors characteristics
   std_acc  = 0.1# Accelerometer standard deviation noise
   std_baro = 0.1     # Barometric Pressure sensor standard deviation noise

   # Define Covariance Matrices
   Q = pow(std_acc,2) * np.array([[0.25*pow(Ts,4), 0.5*pow(Ts,3)],   # Covariance of the process noise
                                  [0.5*pow(Ts,3), pow(Ts,2)]])
   R = np.array([[pow(std_baro,2)]])    # Covariance of the observation noise
               

   ## (0.) Initialize System State Estimate & System State Error Covariance
   x_EstPrev = x_EstPrev
   P_EstPrev = P_EstPrev

   ## (1.) Predict System State Estimate & System State Error Covariance
   # - - -  PREDICTION STEP  - - - #
   A = np.array([[1.0, Ts],
                 [0.0,    1.0]])
   B = np.array([[0.5*pow(Ts,2)], 
                 [Ts]])

   x_Pred = A @ x_EstPrev + B * acc_z     # Predicted a priori state estimate
   P_Pred = A @ P_EstPrev @ A.T + Q       # Predicted a priori estimate covariance

   # Update  x, P
   x_PredPrev = x_Pred
   P_PredPrev = P_Pred

   ## (2.) Compute  Kalman Gain
   # - - -  CORRECTION STEP  - - - #
   H = np.array([[1, 0]])

   K = P_Pred @ H.T @ inv(H @ P_Pred @ H.T + R) 

   ## (3.) Compute the estimate
   y = h_baro - x_PredPrev[0]
   x_NextEst = x_PredPrev + K * y
   x_Kalman = x_NextEst

   ## Compute the error covariance
   P_NextEst = P_PredPrev - K @ H @ P_PredPrev 
   
   return x_Kalman, P_NextEst

