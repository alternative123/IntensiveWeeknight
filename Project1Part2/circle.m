function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

timeend = 9;
w = (2*pi/timeend); % rotation speed
r = 5; % radius of circle

if t < timeend
    pos = [  r*cos(w*t);
             r*sin(w*t);
             (r/(2*timeend))*t ];

    vel = [ -r*w*sin(w*t);
             r*w*cos(w*t);
             r/(2*timeend)  ];

    acc = [ -r*w^2*cos(w*t);
            -r*w^2*sin(w*t);
             0               ];
else
    pos = [0*r; 0; r/2];
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
