# Kalman Filter
Kalman Filters are used for state estimation in control systems when noisy measurements are present. This repository includes an implementation of the algorithm in Python and also a Jupyter Notebook for testing in real data for altitude estimation of a quadrotor. The algorithm was applied in the quad using a sensor-fusion technique by blending the measurements from the barometric pressure sensor and the accelerometer to provide altitude (relative height of the quad) estimation. 

## Kalman Algorithm

### Initialization
0. Start

1. Initialize the state estimate: $`\hat{x}_{0|0}`$ and the error covariance estimate: $`P_{0|0}`$

### Prediction Step

2. Predict the state for the next time step:<br>
   $`\hat{x}_{k|k-1} = A_k \hat{x}_{k-1|k-1} + B_k u_k`$
   
3. Predict the error covariance for the next time step:<br>
   $`P_{k|k-1} = A_k P_{k-1|k-1} A_k^T + Q_k`$

### Correct Step

4. Compute the Kalman Gain:<br>
   $`K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}`$

5. Update the state estimate:<br>
   $`\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k(z_k - H_k \hat{x}_{k|k-1})`$

6. Update the error covariance estimate: <br>
   $`P_{k|k} = (I - K_k H_k)P_{k|k-1}`$
   
      Go to 2


The matrices and vectors in the above equations are defined as follows:

- $`A_k`$: State transition matrix for time step $`k`$
- $`B_k`$: Input control matrix for time step $`k`$
- $`u_k`$: Control input at time step $`k`$
- $`Q_k`$: Process noise covariance matrix at time step $`k`$
- $`H_k`$: Measurement matrix at time step $`k`$
- $`R_k`$: Measurement noise covariance matrix at time step $`k`$
- $`z_k`$: Measurement vector at time step $`k`$
- $`I`$: Identity matrix
