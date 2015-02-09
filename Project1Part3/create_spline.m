function spline = create_spline(path)
% Create spline object
% X,Y,Z are the points we want to interpolate (1 x n)
% T is the times we want to be at those points (1 x n)

% Figure out times
avgvel = 2.5; % HOW DO WE KNOW THIS??? This is just a guess
pathlengths=sqrt(sum(diff(path).^2,2));
T = [0; cumsum(pathlengths/avgvel)]';

X=path(:,1);
Y=path(:,2);
Z=path(:,3);
n = length(T); % Number of points
m = 3+1; % Order of the polynomial
% Number of position constraints: 
% A = zeros(m*(n-1));
% bx = zeros(m*(n-1),1);
% by = zeros(m*(n-1),1);
% bz = zeros(m*(n-1),1);
A = zeros(m*(n-1)+2,m*(n-1));
bx = zeros(m*(n-1)+2,1);
by = zeros(m*(n-1)+2,1);
bz = zeros(m*(n-1)+2,1);

% Positions
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
% A(end-1,1:m)         = [3*T(1)^2 2*T(1) 1 0];
% A(end,(1:m)+m*(n-2)) = [3*T(n)^2 2*T(n) 1 0];
% Endpoint velocities - only velocities and accelerations constrained
A(end-3,1:m)           = [3*T(1)^2 2*T(1) 1 0];
A(end-2,(1:m)+m*(n-2)) = [3*T(n)^2 2*T(n) 1 0];

% Endpoint accelerations (overconstraining system)
A(end-1,1:m)         = [6*T(1) 2 0 0];
A(end,(1:m)+m*(n-2)) = [6*T(n) 2 0 0];


% Compute coeffcients and create the object
spline.T = T;
spline.Cx = reshape(A\bx, m, [])';
spline.Cy = reshape(A\by, m, [])';
spline.Cz = reshape(A\bz, m, [])';
spline.vecpos = @(t) evaluate_pos(t,spline);
spline.vecvel = @(t) evaluate_vel(t,spline);
spline.vecacc = @(t) evaluate_acc(t,spline);
spline.pos = @(t) single_pos(t,spline);
spline.vel = @(t) single_vel(t,spline);
spline.acc = @(t) single_acc(t,spline);


end

function [ px, py, pz ] = evaluate_pos(t,spline)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(spline.T,size(t,1),1) <= repmat(t,1,size(spline.T,2)),2),[],2);
t_ind = min(t_ind,size(spline.Cx,1));

% Create the appropriate positions using the coefficients
px = sum(spline.Cx(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);
py = sum(spline.Cy(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);
pz = sum(spline.Cz(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);

end

function [ vx, vy, vz ] = evaluate_vel(t,spline)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(spline.T,size(t,1),1) <= repmat(t,1,size(spline.T,2)),2),[],2);
t_ind = min(t_ind,size(spline.Cx,1));

% Create the appropriate positions using the coefficients
vx = sum(spline.Cx(t_ind,:).*[3*t.^2 2*t ones(size(t)) zeros(size(t))], 2);
vy = sum(spline.Cy(t_ind,:).*[3*t.^2 2*t ones(size(t)) zeros(size(t))], 2);
vz = sum(spline.Cz(t_ind,:).*[3*t.^2 2*t ones(size(t)) zeros(size(t))], 2);

end

function [ ax, ay, az ] = evaluate_acc(t,spline)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(spline.T,size(t,1),1) <= repmat(t,1,size(spline.T,2)),2),[],2);
t_ind = min(t_ind,size(spline.Cx,1));

% Create the appropriate positions using the coefficients
ax = sum(spline.Cx(t_ind,:).*[6*t 2*ones(size(t)) zeros(size(t)) zeros(size(t))], 2);
ay = sum(spline.Cy(t_ind,:).*[6*t 2*ones(size(t)) zeros(size(t)) zeros(size(t))], 2);
az = sum(spline.Cz(t_ind,:).*[6*t 2*ones(size(t)) zeros(size(t)) zeros(size(t))], 2);

end

% Evaluate only one thing
function p = single_pos(t,spline)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(spline.T <= t, 1, 'last');
t_ind = min(t_ind,size(spline.Cx,1));

% Create the appropriate positions using the coefficients
p = zeros(3,1);
p(1) = spline.Cx(t_ind,:)*[t^3 t^2 t 1]';
p(2) = spline.Cy(t_ind,:)*[t^3 t^2 t 1]';
p(3) = spline.Cz(t_ind,:)*[t^3 t^2 t 1]';

end

function v = single_vel(t,spline)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(spline.T <= t, 1, 'last');
t_ind = min(t_ind,size(spline.Cx,1));

% Create the appropriate positions using the coefficients
v = zeros(3,1);
v(1) = spline.Cx(t_ind,:)*[3*t^2 2*t 1 0]';
v(2) = spline.Cy(t_ind,:)*[3*t^2 2*t 1 0]';
v(3) = spline.Cz(t_ind,:)*[3*t^2 2*t 1 0]';

end

function a = single_acc(t,spline)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(spline.T <= t, 1, 'last');
t_ind = min(t_ind,size(spline.Cx,1));

% Create the appropriate positions using the coefficients
a = zeros(3,1);
a(1) = spline.Cx(t_ind,:)*[6*t 2 0 0]';
a(2) = spline.Cy(t_ind,:)*[6*t 2 0 0]';
a(3) = spline.Cz(t_ind,:)*[6*t 2 0 0]';

end