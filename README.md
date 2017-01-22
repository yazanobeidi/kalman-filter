# Kalman filter
Recursive and iterative implementations of the Kalman Filter for n dimensions.

Python: iterative, uses Pandas

MATLAB: recursive

https://en.wikipedia.org/wiki/Kalman_filter

## Usage

The Kalman filter is a single recursive function:

### MATLAB

```[ output ] = kalmanf(zk, A, xk, Pk, B, uk, wk, Q, R, H, output, t)```

Try running runkalman.m for an example.

### Python

```kalman_filter(zk xk, A,, B, Pk, uk, wk, Q, R, H)```

INPUTS

* zk (pandas timeseries): input data
* xk (np.array): a priori state estimate vector
* A (np.matrix): state transition coefficient matrix
* B (np.matrix): control coefficient matrix
* Pk (np.matrix): prediction covariance matrix
* uk (np.array): control signal vector
* wk (float): process noise (has covariance Q)
* Q (float): process covariance
* R (float): measurement covariance
* H (np.matrix):  transformation matrix

OUTPUTS

* output (np.array): kalman filtered data

EXAMPLE

![Alt text](kalman_filtering_1.png?raw=true "Title")

## License
Copyright 2015, 2016 Yazan Obeidi (Apache)
