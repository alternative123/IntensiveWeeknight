function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

timeend = 10;
w = (2*pi); % rotation speed
% Using a solved qunitic spline
theta = 6*(t/timeend)^5 - 15*(t/timeend)^4 + 10*(t/timeend)^3;
dtheta = (30*(t)^4/timeend^5 - 60*(t)^3/timeend^4 + 30*(t)^2/timeend^3);
d2theta = (120*(t)^3/timeend^5 - 180*(t)^2/timeend^4 + 60*(t)/timeend^3);
r = 5; % radius of circle

if t < timeend
    pos = [  r*cos(w*theta);
             r*sin(w*theta);
             (r/2)*theta ];

    vel = [ -r*w*sin(w*theta)*dtheta;
             r*w*cos(w*theta)*dtheta;
             (r/2)*dtheta  ];

    acc = [ -r*w^2*cos(w*theta)*dtheta^2 - r*w*sin(w*theta)*d2theta;
            -r*w^2*sin(w*theta)*dtheta^2 + r*w*cos(w*theta)*d2theta;
             (r/2)*d2theta  ];
else
    pos = [r; 0; r/2];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
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
