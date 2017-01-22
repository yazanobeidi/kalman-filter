"""Copyright (c) <2016> <Yazan Obeidi>

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to
do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS 
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER 
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import numpy as np
import pandas as pd

def kalman_filter(zk, xk, A=np.matrix(1), B=np.matrix(0), Pk=1, 
                    uk=np.array(0), wk=0, Q=0.1, R=1, H=np.matrix(1)):
    """Performs Kalman Filtering on pandas timeseries data.
    :param: zk (pandas timeseries): input data
    :param: xk (np.array): a priori state estimate vector
    :param: A (np.matrix): state transition coefficient matrix
    :param: B (np.matrix): control coefficient matrix
    :param: Pk (np.matrix): prediction covariance matrix
    :param: uk (np.array): control signal vector
    :param: wk (float): process noise (has covariance Q)
    :param: Q (float): process covariance
    :param: R (float): measurement covariance
    :param: H (np.matrix):  transformation matrix
    :return: output (np.array): kalman filtered data
    """
    output = np.zeros(len(zk))
    for k, t in enumerate(zk.index):
        # time update (prediction)
        xk = A*xk + B*uk + wk # Predict state
        zk_pred = H*xk # Predict measurement
        Pk = A*Pk*A.T + Q # Predict error covariance
        # measurement update (correction)
        vk = zk[t] - zk_pred # Measurement residual
        S = (H*Pk*H.T) + R # Prediction covariance
        Kk = (Pk*H.T) / S # Compute Kalman Gain
        xk = xk + (Kk * vk) # Update estimate with gain * residual
        Pk = (1 - Kk*H)*Pk # Update error covariance
        output[k] = xk.item()
    return output