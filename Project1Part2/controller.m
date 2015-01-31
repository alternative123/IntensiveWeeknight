function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Naming conventions:
% d stands for 'dot', dd stands for 'double dot' (i.e. derivatives)
% y_a means 'y sub a'
if any(isnan(qd{qn}.pos))
    fprintf('Got here\n')
end

%k_pz = 10;
%k_dz = 3.75
% Control parameters
Kp = diag([1,1,1]);
Kd = diag([1,1,1]);
Kp_angle = diag([1,1,1]);
Kd_angle = diag([1,1,1]);
% k_pphi = 10;
% k_dphi = 5;

% Error terms
% Position error
err = (qd{qn}.pos_des - qd{qn}.pos);
% Compute error orthogonal to trajectory
if norm(qd{qn}.vel_des) > 0
    errp = err - ((qd{qn}.vel_des'*err)/(qd{qn}.vel_des'*qd{qn}.vel_des))*qd{qn}.vel_des;
else % Not sure what to do here
    errp = err;
end
    
% Velocity Error
errv = (qd{qn}.vel_des - qd{qn}.vel);

% Calculate desired accelerations
rdd_des = qd{qn}.acc_des + Kd*errv + Kp*errp;

% Desired roll, pitch and yaw
psi_des = qd{qn}.yaw_des;
phi_des = (1/params.grav)*(rdd_des(1)*sin(psi_des) - rdd_des(2)*cos(psi_des));
theta_des = (1/params.grav)*(rdd_des(1)*cos(psi_des) + rdd_des(2)*sin(psi_des));
angle_des = [ phi_des; theta_des; psi_des ];
anglev_des = [0; 0; qd{qn}.yawdot_des];
          
% Control signals
u1 = params.mass*(params.grav + rdd_des(3));
u2 = Kp_angle*(angle_des - qd{qn}.euler) + Kd_angle*(anglev_des - qd{qn}.omega);

% Thurst
F    = u1;
% Moment
M    = u2; % You should fill this in

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
