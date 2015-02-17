%% Cubic spline
% Create constants for the spline
X    = [ -1;   0;   1 ];
Y    = [  0;   2;   0 ];
Xdot = [ -1; nan; nan ]; % Don't know the last two...
Ydot = [ -5; nan; nan ]; % Don't know the last two...
T    = [  0;   5;   6 ];

A = zeros(8,8); % 8x8 coefficient matrix
bx = zeros(8,1); % The x-direction result vector
by = zeros(8,1); % The y-direction result vector
% Time 1 constraints
A(1,1:4) =  [T(1)^3 T(1)^2 T(1) 1]; % Positions
bx(1)    =  X(1);
by(1)    =  Y(1);
A(2,1:4) =  [3*T(1)^2 2*T(1) 1 0]; % Velocities
bx(2)    =  Xdot(1);
by(2)    =  Ydot(1);

% Time 2 constraints
A(3,1:4) =  [T(2)^3, T(2)^2, T(2), 1]; % Final position for first segment
bx(3)    =  X(2);
by(3)    =  Y(2);
A(4,5:8) =  [T(2)^3, T(2)^2, T(2), 1]; % Initial position for second
bx(4)    =  X(2);
by(4)    =  Y(2);
A(5,1:4) =  [3*T(2)^2, 2*T(2), 1, 0]; % Velocities must be the
A(5,5:8) = -[3*T(2)^2, 2*T(2), 1, 0]; %   same for both segments
bx(5)    =  0;
by(5)    =  0;
A(6,1:4) =  [6*T(2), 2, 0, 0]; % Accelerations must be the
A(6,5:8) = -[6*T(2), 2, 0, 0]; %   same for both segments
bx(6)    =  0;
by(6)    =  0;

% Time 3 constraints
A(7,5:8) =  [T(3)^3, T(3)^2, T(3), 1]; % Final position for first segment
bx(7)    =  X(3);
by(7)    =  Y(3);
A(8,5:8) =  [3*T(3)^2, 2*T(3), 1, 0]; % Velocity vanishes at the end
bx(8)    =  0;
by(8)    =  0;

% Compute the coefficents
Cx = A \ bx;
Cy = A \ by;
fprintf('Coefficients for x, segment 1: (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cx(1),Cx(2),Cx(3),Cx(4));
fprintf('Coefficients for y, segment 1: (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cy(1),Cy(2),Cy(3),Cy(4));
fprintf('Coefficients for x, segment 2: (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cx(5),Cx(6),Cx(7),Cx(8));
fprintf('Coefficients for y, segment 2 (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cy(5),Cy(6),Cy(7),Cy(8));

scatter(X,Y)
t1 = (T(1):0.1:T(2))';
xt1 = [ t1.^3 t1.^2 t1.^1 ones(size(t1)) ]*Cx(1:4);
yt1 = [ t1.^3 t1.^2 t1.^1 ones(size(t1)) ]*Cy(1:4);
t2 = (T(2):0.1:T(3))';
xt2 = [ t2.^3 t2.^2 t2.^1 ones(size(t2)) ]*Cx(5:8);
yt2 = [ t2.^3 t2.^2 t2.^1 ones(size(t2)) ]*Cy(5:8);
hold on;
plot(xt1,yt1)
plot(xt2,yt2)

%% Septic spline

% Create constants for the spline
X    = [ -1;   0;   1 ];
Y    = [  0;   2;   0 ];
Xdot = [ -1; nan; nan ]; % Don't know the last two...
Ydot = [ -5; nan; nan ]; % Don't know the last two...
T    = [  0;   5;   6 ];

A = zeros(16,16); % 16x16 coefficient matrix
bx = zeros(16,1); % The x-direction result vector
by = zeros(16,1); % The y-direction result vector
seg1 = 1:8;
seg2 = 9:16;
% Time 1 constraints
% Initial positions
A(1,seg1)  =  [T(1)^7, T(1)^6, T(1)^5, T(1)^4, T(1)^3, T(1)^2, T(1), 1];
bx(1)      =  X(1);
by(1)      =  Y(1);
% Initial velocity
A(2,seg1)  =  [7*T(1)^6, 6*T(1)^5, 5*T(1)^4 4*T(1)^3 3*T(1)^2 2*T(1) 1 0];
bx(2)      =  Xdot(1);
by(2)      =  Ydot(1);
% Initial acceleration
A(3,seg1)  =  [42*T(1)^5, 30*T(1)^4, 20*T(1)^3 12*T(1)^2 6*T(1) 2 0 0];
bx(3)      =  0;
by(3)      =  0;
% Initial jerk
A(4,seg1)  =  [210*T(1)^4, 120*T(1)^3, 60*T(2)^2 24*T(2) 6 0 0 0];
bx(4)      =  0;
by(4)      =  0;

% Time 2 constraints
% Final position for first segment
A(5,seg1)  =  [T(2)^7, T(2)^6, T(2)^5, T(2)^4, T(2)^3, T(2)^2, T(2), 1];
bx(5)      =  X(2);
by(5)      =  Y(2);
% Initial position for second
A(6,seg2)  =  [T(2)^7, T(2)^6, T(2)^5, T(2)^4, T(2)^3, T(2)^2, T(2), 1];
bx(6)      =  X(2);
by(6)      =  Y(2);
% Velocities must be the same for both segments
A(7,seg1)  =  [7*T(2)^6, 6*T(2)^5, 5*T(2)^4 4*T(2)^3 3*T(2)^2 2*T(2) 1 0];
A(7,seg2)  = -[7*T(2)^6, 6*T(2)^5, 5*T(2)^4 4*T(2)^3 3*T(2)^2 2*T(2) 1 0];
bx(7)      =  0;
by(7)      =  0;
% Accelerations must be the same for both segments
A(8,seg1)  =  [42*T(2)^5, 30*T(2)^4, 20*T(2)^3 12*T(2)^2 6*T(2) 2 0 0];
A(8,seg2)  = -[42*T(2)^5, 30*T(2)^4, 20*T(2)^3 12*T(2)^2 6*T(2) 2 0 0];
bx(8)      =  0;
by(8)      =  0;
% Jerk must be the same for both segments
A(9,seg1)  =  [210*T(2)^4, 120*T(2)^3, 60*T(2)^2 24*T(2) 6 0 0 0];
A(9,seg2)  = -[210*T(2)^4, 120*T(2)^3, 60*T(2)^2 24*T(2) 6 0 0 0];
bx(9)      =  0;
by(9)      =  0;
% 'Snap' must be the same for both segments
A(10,seg1) =  [840*T(2)^3, 360*T(2)^2, 120*T(2), 24, 0, 0, 0, 0];
A(10,seg2) = -[840*T(2)^3, 360*T(2)^2, 120*T(2), 24, 0, 0, 0, 0];
bx(10)     =  0;
by(10)     =  0;
% 'Crackle' must be the same for both segments
A(11,seg1) =  [2520*T(2)^2, 720*T(2), 120, 0, 0, 0, 0, 0];
A(11,seg2) = -[2520*T(2)^2, 720*T(2), 120, 0, 0, 0, 0, 0];
bx(11)     =  0;
by(11)     =  0;
% 'Pop' must be the same for both segments
A(12,seg1) =  [5040*T(2), 720, 0, 0, 0, 0, 0, 0];
A(12,seg2) = -[5040*T(2), 720, 0, 0, 0, 0, 0, 0];
bx(12)     =  0;
by(12)     =  0;


% Time 3 constraints
% Final position for first segment
A(13,seg2) =  [T(3)^7, T(3)^6, T(3)^5, T(3)^4, T(3)^3, T(3)^2, T(3), 1];
bx(13)     =  X(3);
by(13)     =  Y(3);
% Velocity vanishes at the end
A(14,seg2) =  [7*T(3)^6, 6*T(3)^5, 5*T(3)^4, 4*T(3)^3, 3*T(3)^2, 2*T(3), 1, 0];
bx(14)     =  0;
by(14)     =  0;
% Final acceleration
A(15,seg2) =  [42*T(3)^5, 30*T(3)^4, 20*T(3)^3, 12*T(3)^2, 6*T(3), 2, 0, 0];
bx(15)     =  0;
by(15)     =  0;
% Final jerk
A(16,seg1)  =  [210*T(3)^4, 120*T(3)^3, 60*T(3)^2 24*T(2) 6 0 0 0];
bx(16)      =  0;
by(16)      =  0;

% Compute the coefficents
Cx = A \ bx;
Cy = A \ by;
fprintf('Coefficients for x, segment 1:\n (%0.4f)*t^7 + (%0.4f)*t^6 + (%0.4f)*t^5 + (%0.4f)*t^4 + (%0.4f)*t^3 + (%0.4f)*t^2 + (%0.4f)*t + (%0.4f)\n',Cx(1),Cx(2),Cx(3),Cx(4),Cx(5),Cx(6),Cx(7),Cx(8));
fprintf('Coefficients for y, segment 1:\n (%0.4f)*t^7 + (%0.4f)*t^6 + (%0.4f)*t^5 + (%0.4f)*t^4 + (%0.4f)*t^3 + (%0.4f)*t^2 + (%0.4f)*t + (%0.4f)\n',Cy(1),Cy(2),Cy(3),Cy(4),Cy(5),Cy(6),Cy(7),Cy(8));
fprintf('Coefficients for x, segment 2:\n (%0.4f)*t^7 + (%0.4f)*t^6 + (%0.4f)*t^5 + (%0.4f)*t^4 + (%0.4f)*t^3 + (%0.4f)*t^2 + (%0.4f)*t + (%0.4f)\n',Cx(9),Cx(10),Cx(11),Cx(12),Cx(13),Cx(14),Cx(15),Cx(16));
fprintf('Coefficients for y, segment 2:\n (%0.4f)*t^7 + (%0.4f)*t^6 + (%0.4f)*t^5 + (%0.4f)*t^4 + (%0.4f)*t^3 + (%0.4f)*t^2 + (%0.4f)*t + (%0.4f)\n',Cy(9),Cy(10),Cy(11),Cy(12),Cy(13),Cy(14),Cy(15),Cy(16));

scatter(X,Y)
t1 = (T(1):0.1:T(2))';
xt1 = [ t1.^7 t1.^6 t1.^5 t1.^4 t1.^3 t1.^2 t1.^1 ones(size(t1)) ]*Cx(seg1);
yt1 = [ t1.^7 t1.^6 t1.^5 t1.^4 t1.^3 t1.^2 t1.^1 ones(size(t1)) ]*Cy(seg1);
t2 = (T(2):0.1:T(3))';
xt2 = [ t2.^7 t2.^6 t2.^5 t2.^4 t2.^3 t2.^2 t2.^1 ones(size(t2)) ]*Cx(seg2);
yt2 = [ t2.^7 t2.^6 t2.^5 t2.^4 t2.^3 t2.^2 t2.^1 ones(size(t2)) ]*Cy(seg2);
hold on;
plot(xt1,yt1,'b')
plot(xt2,yt2,'b')

%% Quintic Spline

% Create constants for the spline
X    = [ -1;   0;   1 ];
Y    = [  0;   2;   0 ];
Xdot = [ -1; nan; nan ]; % Don't know the last two...
Ydot = [ -5; nan; nan ]; % Don't know the last two...
T    = [  0;   5;   6 ];

A = zeros(12,12); % 12x12 coefficient matrix
bx = zeros(12,1); % The x-direction result vector
by = zeros(12,1); % The y-direction result vector
seg1 = 1:6;
seg2 = 7:12;
% Time 1 constraints
% Initial positions
A(1,seg1)  =  [T(1)^5, T(1)^4, T(1)^3, T(1)^2, T(1), 1]; 
bx(1)      =  X(1);
by(1)      =  Y(1);
% Initial velocity
A(2,seg1)  =  [5*T(1)^4 4*T(1)^3 3*T(1)^2 2*T(1) 1 0];
bx(2)      =  Xdot(1);
by(2)      =  Ydot(1);
% Initial acceleration
A(3,seg1)  =  [20*T(1)^3 12*T(1)^2 6*T(1) 2 0 0];
bx(3)      =  0;
by(3)      =  0;

% Time 2 constraints
% Final position for first segment
A(4,seg1)  =  [T(2)^5, T(2)^4, T(2)^3, T(2)^2, T(2), 1];
bx(4)      =  X(2);
by(4)      =  Y(2);
% Initial position for second
A(5,seg2)  =  [T(2)^5, T(2)^4, T(2)^3, T(2)^2, T(2), 1];
bx(5)      =  X(2);
by(5)      =  Y(2);
% Velocities must be the same for both segments
A(6,seg1)  =  [5*T(2)^4 4*T(2)^3 3*T(2)^2 2*T(2) 1 0];
A(6,seg2)  = -[5*T(2)^4 4*T(2)^3 3*T(2)^2 2*T(2) 1 0];
bx(6)      =  0;
by(6)      =  0;
% Accelerations must be the same for both segments
A(7,seg1)  =  [20*T(2)^3 12*T(2)^2 6*T(2) 2 0 0];
A(7,seg2)  = -[20*T(2)^3 12*T(2)^2 6*T(2) 2 0 0];
bx(7)      =  0;
by(7)      =  0;
% Jerk must be the same for both segments
A(8,seg1)  =  [60*T(2)^2 24*T(2) 6 0 0 0];
A(8,seg2)  = -[60*T(2)^2 24*T(2) 6 0 0 0];
bx(8)      =  0;
by(8)      =  0;
% Snap must be the same for both segments
A(9,seg1)  =  [120*T(2) 24 0 0 0 0];
A(9,seg2)  = -[120*T(2) 24 0 0 0 0];
bx(9)      =  0;
by(9)      =  0;

% Time 3 constraints
% Final position for first segment
A(10,seg2) =  [T(3)^5, T(3)^4, T(3)^3, T(3)^2, T(3), 1];
bx(10)     =  X(3);
by(10)     =  Y(3);
% Velocity vanishes at the end
A(11,seg2) =  [5*T(3)^4 4*T(3)^3 3*T(3)^2 2*T(3) 1 0];
bx(11)     =  0;
by(11)     =  0;
% Final acceleration
A(12,seg2) =  [20*T(3)^3 12*T(3)^2 6*T(3) 2 0 0];
bx(12)     =  0;
by(12)     =  0;


% Compute the coefficents
Cx = A \ bx;
Cy = A \ by;
fprintf('Coefficients for x, segment 1: (%f)*t^5 + (%f)*t^4 + (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cx(1),Cx(2),Cx(3),Cx(4),Cx(5),Cx(6));
fprintf('Coefficients for y, segment 1: (%f)*t^5 + (%f)*t^4 + (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cy(1),Cy(2),Cy(3),Cy(4),Cy(5),Cy(6));
fprintf('Coefficients for x, segment 2: (%f)*t^5 + (%f)*t^4 + (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cx(7),Cx(8),Cx(9),Cx(10),Cx(11),Cx(12));
fprintf('Coefficients for y, segment 2: (%f)*t^5 + (%f)*t^4 + (%f)*t^3 + (%f)*t^2 + (%f)*t + (%f)\n',Cy(7),Cy(8),Cy(9),Cy(10),Cy(11),Cy(12));

scatter(X,Y)
t1 = (T(1):0.1:T(2))';
xt1 = [ t1.^5 t1.^4 t1.^3 t1.^2 t1.^1 ones(size(t1)) ]*Cx(1:6);
yt1 = [ t1.^5 t1.^4 t1.^3 t1.^2 t1.^1 ones(size(t1)) ]*Cy(1:6);
t2 = (T(2):0.1:T(3))';
xt2 = [ t2.^5 t2.^4 t2.^3 t2.^2 t2.^1 ones(size(t2)) ]*Cx(7:12);
yt2 = [ t2.^5 t2.^4 t2.^3 t2.^2 t2.^1 ones(size(t2)) ]*Cy(7:12);
hold on;
plot(xt1,yt1)
plot(xt2,yt2)


%% Positions
for i = 1:n-1
    j = i+1;
    % Coefficients matrix
    A(2*i-1,(1:m)+m*(i-1)) = [T(i)^3 T(i)^2 T(i) 1];
    A(2*i  ,(1:m)+m*(i-1)) = [T(j)^3 T(j)^2 T(j) 1];

    % b for the Ax = b for the x,y,z parts
    bx(2*i-1) = X(i);
    bx(2*i)   = X(j);
    by(2*i-1) = Y(i);
    by(2*i)   = Y(j);
    bz(2*i-1) = Z(i);
    bz(2*i)   = Z(j);
end

% Velocities at the boundaries
offset = 2*(n-1);
for i = 1:n-2
    j = i+1;
    % Coefficients matrix
    A(i+offset,(1:m)+m*(i-1)) = [3*T(j)^2 2*T(j) 1 0];
    A(i+offset,(1:m)+m*(i)) = -[3*T(j)^2 2*T(j) 1 0];
end

% Accelerations at the boundaries
offset = 2*(n-1)+(n-2);
for i = 1:n-2
    j = i+1;
    % Coefficients matrix
    A(i+offset,(1:m)+m*(i-1)) = [6*T(j) 2 0 0];
    A(i+offset,(1:m)+m*(i)) = -[6*T(j) 2 0 0];
end

% Endpoint velocities - only velocities constrained
A(end-1,1:m)         = [3*T(1)^2 2*T(1) 1 0];
A(end,(1:m)+m*(n-2)) = [3*T(n)^2 2*T(n) 1 0];


%%

R = [-0.3038,  -0.6313, -0.7135;
-0.9332,  0.3481, 0.0893;
0.1920, 0.6930, -0.6949 ];

theta = acos((trace(R)-1)/2);

omega = [ R(3,2) - R(2,3);
          R(1,3) - R(3,1);
          R(2,1) - R(1,2) ] / (2*sin(theta));

disp(theta)
disp(omega)

hat = @(w) [ 0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];



