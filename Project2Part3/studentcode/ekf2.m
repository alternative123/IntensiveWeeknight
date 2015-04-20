function [X, Z] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; roll; pitch; yaw; other states you use]
%     we will only take the first 9 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement 

n = 15; % Size of our state
m = 9; % Size of our measurements

% Persistent variables
persistent mu
persistent Sigma
persistent time_prev

if isempty(mu)
    mu = zeros(n,1);
    % Set the bias
    mu(10) = -0.0000120;
    mu(11) = -0.0000260;
    mu(12) = -0.0010920;
    mu(13) = 0.1320661;
    mu(14) = 0.0216428;
    mu(15) = 9.7239545 - 9.81;
    Sigma = eye(n);
    time_prev = 0;
    clear estimate_vel
end

% Set up helper and variables
R = @(q) transpose([cos(q(3))*cos(q(2)) - sin(q(1))*sin(q(3))*sin(q(2)), ...
     cos(q(2))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)), ...
     -cos(q(1))*sin(q(2)); ...
     -cos(q(1))*sin(q(3)),...
     cos(q(1))*cos(q(3)), ...
     sin(q(1));...
     cos(q(3))*sin(q(2)) + cos(q(2))*sin(q(1))*sin(q(3)),...
     sin(q(3))*sin(q(2)) - cos(q(3))*cos(q(2))*sin(q(1)),...
     cos(q(1))*cos(q(2))]);

G = @(q) [cos(q(2)) 0 -cos(q(1))*sin(q(2));
              0     1         0;
          sin(q(2)) 0  cos(q(1))*cos(q(2)) ];
K = varargin{1};
tagsX = varargin{2};
tagsY = varargin{3};
g = [0,0,9.81]';

dt = sensor.t - time_prev;
time_prev = sensor.t;

w_m = sensor.omg;
a_m = sensor.acc;

% EKF part
% Process model
xdot = [ mu(7:9);
         inv(G(mu(4:6)))*(w_m - mu(10:12));
         -g+R(mu(4:6))*(a_m - mu(13:15));
         zeros(3,1);
         zeros(3,1) ];

J = ekf2jacobian(mu,w_m,a_m,zeros(n,1));
F = eye(n) + dt*J;
U = ekf2noisejacobian(mu);
V = dt*U;
Q = eye(n); % TODO: tune this
Q(10,10) = 0.0000039;
Q(11,11) = 0.0000050;
Q(12,12) = 0.0000044;
Q(13,13) = 0.0002536;
Q(14,14) = 0.0001434;
Q(15,15) = 0.0001348;

% Predition
mu_pred = mu + dt*xdot;
Sigma_pred = F*Sigma*F' + V*Q*V';

% Update
if sensor.is_ready && ~isempty(sensor.id)
    % Get Measurements
    [pos, eul, R, T] = estimate_pose(sensor, K, tagsX, tagsY);
    [vel, ~] = estimate_vel(sensor, K, tagsX, tagsY, R, T);
    Z = [pos; eul; vel];

    % Measurement model - it is so nice to be linear isn't it?
    C = eye(m,n);
    W = eye(m);
    R = eye(m);

    % Now preform the update
    K = Sigma_pred*C'/(C*Sigma_pred*C' + W*R*W'); % Kalman gain
    mu = mu_pred + K*(Z - C*mu_pred);
    Sigma = Sigma_pred - K*C*Sigma_pred;
else
    % No measurements - just use the prediction, we can't do better
    mu = mu_pred;
    Sigma = Sigma_pred;
    Z = zeros(m,1);
end

% X = mu;
% Reorder to get output right
X = [mu(1:3); mu(7:9); mu(4:6); mu(10:end)];

end
