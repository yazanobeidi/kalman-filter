%%% Kalman Filter Run Script
% This script attemps to explore custom Kalman filter possibilities
% - as opposed to entering these through command

% Create data
x =  [1;2;3;4;5;6;7;8;9;10];
%y = [3; 4.76; 7.1; 8.9; 11; 13; 14.9; 17; 20; 22];
%y2 = [0.39;0.5; 0.48; 0.29; 0.25; 0.32; 0.34; 0.48; 0.41; 0.45];
y3 = [160;158;161;165;146;159;155;157;142;157];
%y4 = [0; 1; 4; 16; 25; 36; 49; 64; 81; 100]; % needs another dimension

% Initialize kalman filter variables
results = []; % initialize results store
t = size(y3); t = t(1); % set recursive counter.. size of your data
xi = 160;%;0]; % initial state vector
Pi = 1;%,0;0,0]; % initial covariance matrix
A = 1; % state transition coefficient matrix
B = -1;%2; % control coefficient matrix
uk = 1;%1; % control signal vector
wk = 0; % process noise vector (for each state parameter)
Q = cov(wk); % process covariance
R = cov(1:1:t,y3);%0.1;%, 0; 0, 0]; % measurement covariance
R = R(1,1); % use if there is a relation between x and y
%R = R(2,2); % use if there is y is independent e.g. fixed value
H = 1;%,1;1,1]; % transformation matrix

% Plot data
plot(x,y3,'bx'), hold on

% Perform Kalman filter
results = kalmanf(y3, A, xi, Pi, B, uk, wk, Q, R, H, results, t);

% Plot filter regression output
plot(x,results,'g--')

% Compare to linear regression
%fitresult = fit(x,y3,'poly1');
%plot(fitresult)

hold off
ylim([0 inf]) % adjust y axis limits

clear A xi Pi B uk wk Q R H t fitresult

% Written by Yazan Obeidi July 2015, Princess Margaret Cancer Centre
