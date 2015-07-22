function [ output ] = kalmanf(zk, A, xk, Pk, B, uk, wk, Q, R, H, output, t)
%%%KALMAN FUNCTION
%%% INPUTS
% * zk : raw data
% * A : state transition coefficient matrix
% * xk : state estimate vector
% * Pk : prediction covariance matrix
% * B : control coefficient matrix
% * uk : control signal vector
% * wk : process noise (has covariance Q)
% * Q : process covariance
% * R : measurement covariance
% * H :  transformation matrix
% * t : initially size of zk, decrements as data is processed

% Unessential, sets correct sample col index increment for each iteration
zk_index = size(zk);
zk_index = zk_index(1) - t + 1;

% Time update (prediction)
xk = A*xk + B*uk + wk; % Predict state
zk_pred = H*xk; % Predict measurement (comment out if alt est update used)
Pk = A*Pk*transpose(A) + Q; % Predict error covariance

% Measurement update  (correction)
vk = (zk(zk_index) - zk_pred); % Measurement residual
S = (H*Pk*transpose(H) + R); % Prediction covariance
Kk = Pk*transpose(H) / S ; % Compute Kalman Gain
xk = xk + Kk * (vk); % Update estimate with gain * residual
%xk = xk + Kk * (zk(zk_index) - H*xk); % Update estimate alternative
Pk = (1 - Kk*H)*Pk; % Update error covariance simplified
%Pk = Pk - Kk*S*transpose(Kk); % Update error covariance alternative

t = t - 1; % to keep track of iterations
output = [output; vertcat(transpose(xk))]; % collect output to display at end

if t <= 0
    return; % end if no new data remains
else
    output = kalmanf(zk, A, xk, Pk, B, uk, wk, Q, R, H, output, t); % recurse to next sample
end
end

% Written by Yazan Obeidi July 2015, Princess Margaret Cancer Centre

