function [X, Z] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; roll; pitch; yaw; other states you use]
%     we will only take the first 6 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement 

n = 9; % Size of our state
m = 6; % Size of our state

% Persistent variables
persistent mu
persistent Sigma
persistent time_prev

if isempty(mu)
    mu = zeros(n,1);
    Sigma = 3*eye(n);
    time_prev = 0;
end

% Set up helper and variables
G = @(q) [cos(q(2)) 0 -cos(q(1))*sin(q(2));
              0     1         0;
          sin(q(2)) 0  cos(q(1))*cos(q(2)) ];
K = varargin{1};
tagsX = varargin{2};
tagsY = varargin{3};

v_m = vic.vel(1:3);
w_m = vic.vel(4:6);
dt = vic.t - time_prev;
time_prev = vic.t;

% EKF part
% Process model
xdot = [v_m;
        G(mu(4:6))\(w_m - mu(7:9));
        zeros(3,1) ];
J = ekf1jacobian(mu,w_m,zeros(n,1));
F = eye(n) + dt*J;
U = ekf1noisejacobian(mu);
V = dt*U;
Q = eye(n); % TODO: tune this

% Predition
mu_pred = mu + dt*xdot;
Sigma_pred = F*Sigma*F' + V*Q*V';
    
% Update
if sensor.is_ready && ~isempty(sensor.id)
    % Got measurements
    [pos, eul, ~, ~] = estimate_pose(sensor, K,tagsX,tagsY);
    Z = [pos; eul];
    
    % Measurement model - it is so nice to be linear isn't it?
    C = eye(m,n);
    W = eye(m);
    R = eye(m);

    % Do the update
    K = Sigma_pred*C'/(C*Sigma_pred*C' + W*R*W'); % Kalman gain
    mu = mu_pred + K*(Z - C*mu_pred);
    Sigma = Sigma_pred - K*C*Sigma_pred;
else
    mu = mu_pred;
    Sigma = Sigma_pred;
    Z = zeros(m,1);
end

X = mu;

end
