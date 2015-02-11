function sptraj = create_spline(path,map)
% Create spline object
% X,Y,Z are the points we want to interpolate (1 x n)
% T is the times we want to be at those points (1 x n)

% Figure out times
path=optimize_path(path,map,true);
pathlengths=sqrt(sum(diff(path).^2,2));
timelengths = nthroot(pathlengths,3.6);
% totattime = sum(pathlengths)/avgvel;
T = [0; cumsum(timelengths)]';

% Just in case - angles
% vects = diff(path);
% angles = abs(acos((sum(vects(1:end-1,:) .* vects(2:end,:),2)) ./ ...
%    (sqrt(sum(vects(1:end-1,:).^2,2)).*sqrt(sum(vects(2:end,:).^2,2)))));

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
A(end-1,1:m)         = [3*T(1)^2 2*T(1) 1 0];
A(end,(1:m)+m*(n-2)) = [3*T(n)^2 2*T(n) 1 0];
% % Endpoint velocities - only velocities and accelerations constrained
% A(end-3,1:m)           = [3*T(1)^2 2*T(1) 1 0];
% A(end-2,(1:m)+m*(n-2)) = [3*T(n)^2 2*T(n) 1 0];
% 
% % Endpoint accelerations (overconstraining system)
% A(end-1,1:m)         = [6*T(1) 2 0 0];
% A(end,(1:m)+m*(n-2)) = [6*T(n) 2 0 0];


% Compute coeffcients and create the object
sptraj.T = T;
sptraj.Cx = reshape(A\bx, m, [])';
sptraj.Cy = reshape(A\by, m, [])';
sptraj.Cz = reshape(A\bz, m, [])';
sptraj.vecpos = @(t) evaluate_pos(t,sptraj);
sptraj.vecvel = @(t) evaluate_vel(t,sptraj);
sptraj.vecacc = @(t) evaluate_acc(t,sptraj);
sptraj.pos = @(t) single_pos(t,sptraj);
sptraj.vel = @(t) single_vel(t,sptraj);
sptraj.acc = @(t) single_acc(t,sptraj);
sptraj.goal = path(end,:)';

end

function [ px, py, pz ] = evaluate_pos(t,sptraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(sptraj.T,size(t,1),1) <= repmat(t,1,size(sptraj.T,2)),2),[],2);
t_ind = min(t_ind,size(sptraj.Cx,1));

% Create the appropriate positions using the coefficients
px = sum(sptraj.Cx(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);
py = sum(sptraj.Cy(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);
pz = sum(sptraj.Cz(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);

end

function [ vx, vy, vz ] = evaluate_vel(t,sptraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(sptraj.T,size(t,1),1) <= repmat(t,1,size(sptraj.T,2)),2),[],2);
t_ind = min(t_ind,size(sptraj.Cx,1));

% Create the appropriate positions using the coefficients
vx = sum(sptraj.Cx(t_ind,:).*[3*t.^2 2*t ones(size(t)) zeros(size(t))], 2);
vy = sum(sptraj.Cy(t_ind,:).*[3*t.^2 2*t ones(size(t)) zeros(size(t))], 2);
vz = sum(sptraj.Cz(t_ind,:).*[3*t.^2 2*t ones(size(t)) zeros(size(t))], 2);

end

function [ ax, ay, az ] = evaluate_acc(t,sptraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(sptraj.T,size(t,1),1) <= repmat(t,1,size(sptraj.T,2)),2),[],2);
t_ind = min(t_ind,size(sptraj.Cx,1));

% Create the appropriate positions using the coefficients
ax = sum(sptraj.Cx(t_ind,:).*[6*t 2*ones(size(t)) zeros(size(t)) zeros(size(t))], 2);
ay = sum(sptraj.Cy(t_ind,:).*[6*t 2*ones(size(t)) zeros(size(t)) zeros(size(t))], 2);
az = sum(sptraj.Cz(t_ind,:).*[6*t 2*ones(size(t)) zeros(size(t)) zeros(size(t))], 2);

end

% Evaluate only one thing
function p = single_pos(t,sptraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(sptraj.T <= t, 1, 'last');
t_ind = min(t_ind,size(sptraj.Cx,1));

% Create the appropriate positions using the coefficients
p = zeros(3,1);
p(1) = sptraj.Cx(t_ind,:)*[t^3 t^2 t 1]';
p(2) = sptraj.Cy(t_ind,:)*[t^3 t^2 t 1]';
p(3) = sptraj.Cz(t_ind,:)*[t^3 t^2 t 1]';

end

function v = single_vel(t,sptraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(sptraj.T <= t, 1, 'last');
t_ind = min(t_ind,size(sptraj.Cx,1));

% Create the appropriate positions using the coefficients
v = zeros(3,1);
v(1) = sptraj.Cx(t_ind,:)*[3*t^2 2*t 1 0]';
v(2) = sptraj.Cy(t_ind,:)*[3*t^2 2*t 1 0]';
v(3) = sptraj.Cz(t_ind,:)*[3*t^2 2*t 1 0]';

end

function a = single_acc(t,sptraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(sptraj.T <= t, 1, 'last');
t_ind = min(t_ind,size(sptraj.Cx,1));

% Create the appropriate positions using the coefficients
a = zeros(3,1);
a(1) = sptraj.Cx(t_ind,:)*[6*t 2 0 0]';
a(2) = sptraj.Cy(t_ind,:)*[6*t 2 0 0]';
a(3) = sptraj.Cz(t_ind,:)*[6*t 2 0 0]';

end