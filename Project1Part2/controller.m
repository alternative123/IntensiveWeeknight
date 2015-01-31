function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
phi_des = 0;
theta_des = 0;
psi_des = 0;

% Thurst
k_pz = 10;
k_dz = 3.75;
e = (qd{qn}.pos_des - qd{qn}.pos);
edot = (qd{qn}.vel_des - qd{qn}.vel);
F    = params.mass*(params.grav + qd{qn}.acc_des(3) + k_dz*edot(3) + k_pz*e(3));

% Moment
k_pphi = 1;
k_dphi = 1;
M    = 0*[ 0; 0; k_pphi + k_dphi]; % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
