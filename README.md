# kalman filter
This MATLAB function is a recursive implementation of the Kalman Filter

function [ output ] = kalmanf(zk, A, xk, Pk, B, uk, wk, Q, R, H, output, t)

runkalman.m can be called from command for an example

INPUTS
* zk : raw data
* A : state transition coefficient matrix
* xk : state estimate vector
* Pk : prediction covariance matrix
* B : control coefficient matrix
* uk : control signal vector
* wk : process noise (has covariance Q)
* Q : process covariance
* R : measurement covariance
* H :  transformation matrix
* t : initially size of zk, decrements as data is processed

OUTPUTS

* output : resulting data points for each iteration of the filter, can be plotted to see tendancy
