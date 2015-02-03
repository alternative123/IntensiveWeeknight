function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% Interpolating values
timeend = 9;
D = timeend/4; % The segment constant

points = [ 0.0,       0,          0;
           1/4,  sqrt(2),   sqrt(2);
           2/4,        0, 2*sqrt(2);
           3/4, -sqrt(2),   sqrt(2);
           1.0,       0,          0  ]';
% Using a solved qunitic spline
theta = @(t1) 6*(t1)^5 - 15*(t1)^4 + 10*(t1)^3;
dtheta = @(t1) (30*(t1)^4 - 60*(t1)^3 + 30*(t1)^2);
d2theta = @(t1) (120*(t1)^3 - 180*(t1)^2 + 60*(t1));

if t < D
    pos = theta(t/D)*(points(:,2)-points(:,1)) + points(:,1);
    vel = dtheta(t/D)/D*(points(:,2)-points(:,1));
    acc = d2theta(t/D)/D^2*(points(:,2)-points(:,1));
elseif t < 2*D
    pos = theta((t-1*D)/D)*(points(:,3)-points(:,2)) + points(:,2);
    vel = dtheta((t-1*D)/D)/D*(points(:,3)-points(:,2));
    acc = d2theta((t-1*D)/D)/D^2*(points(:,3)-points(:,2));
elseif t < 3*D
    pos = theta((t-2*D)/D)*(points(:,4)-points(:,3)) + points(:,3);
    vel = dtheta((t-2*D)/D)/D*(points(:,4)-points(:,3));
    acc = d2theta((t-2*D)/D)/D^2*(points(:,4)-points(:,3));
elseif t < 4*D
    pos = theta((t-3*D)/D)*(points(:,5)-points(:,4)) + points(:,4);
    vel = dtheta((t-3*D)/D)/D*(points(:,5)-points(:,4));
    acc = d2theta((t-3*D)/D)/D^2*(points(:,5)-points(:,4));
else
    pos = points(:,5);
    vel=[0;0;0];
    acc=[0;0;0];
end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
