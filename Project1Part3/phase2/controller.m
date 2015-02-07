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

%k_pz = 10;
%k_dz = 3.75
% Control parameters
Kp = [20*[1;1];20]; % [19*[1;1];12];
Kd = [33*[1;1];30]; % [33*[1;1];30];

Kp_phi = 3.2/10;
Kd_phi = 1.1/10;

Kp_theta = Kp_phi;
Kd_theta = Kd_phi;

Kp_psi = 0.1;
Kd_psi = 0.1;

% Error terms
% Position error
err = (qd{qn}.pos_des - qd{qn}.pos);
% Compute error orthogonal to trajectory
% if norm(qd{qn}.vel_des) > 0
%     errp = err - ((qd{qn}.vel_des'*err)/(qd{qn}.vel_des'*qd{qn}.vel_des))*qd{qn}.vel_des;
% else % Not sure what to do here
    errp = err;
% end
    
% Velocity Error
errv = (qd{qn}.vel_des - qd{qn}.vel);

% Calculate desired accelerations
rdd_des = qd{qn}.acc_des + Kd.*errv + Kp.*errp;

% Desired roll, pitch and yaw
psi_des = qd{qn}.yaw_des;

phi_des = (1/params.grav)*(rdd_des(1)*sin(qd{qn}.euler(3)) - rdd_des(2)*cos(qd{qn}.euler(3)));
phi_des = sign(phi_des)*min(abs(phi_des),params.maxangle); % Clamp at maximum

theta_des = (1/params.grav)*(rdd_des(1)*cos(qd{qn}.euler(3)) + rdd_des(2)*sin(qd{qn}.euler(3)));
theta_des = sign(theta_des)*min(abs(theta_des),params.maxangle); % Clamp at maximum

% Control signals
u2 = [ Kp_phi*(phi_des-qd{qn}.euler(1)) - Kd_phi*qd{qn}.omega(1);
        Kp_theta*(theta_des-qd{qn}.euler(2)) - Kd_theta*qd{qn}.omega(2);
        Kp_psi*(psi_des-qd{qn}.euler(3)) + Kd_psi*(qd{qn}.yawdot_des-qd{qn}.omega(3)) ];
u1 = params.mass*(params.grav + rdd_des(3));

% Thurst
F    = min(params.maxF,max(params.minF,u1));
% Moment
M    = u2;

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
