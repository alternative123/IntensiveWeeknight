function bbtraj = create_bangbang_trajectory(path,map)
% Create a bang bang trajectory generator. Assumes that the path is
% not optimized
% Optimize path
path = optimize_path(path,map);
% Figure out times
avgvel = 2.4; % HOW DO WE KNOW THIS??? This is just a guess
pathlengths=sqrt(sum(diff(path).^2,2));
timelengths = nthroot(pathlengths,3);
totattime = sum(pathlengths)/avgvel;
T = totattime*[0; cumsum(timelengths)/sum(timelengths)]';

bbtraj.theta = @(t1) 6*(t1).^5 - 15*(t1).^4 + 10*(t1).^3;
bbtraj.dtheta = @(t1) (30*(t1).^4 - 60*(t1).^3 + 30*(t1).^2);
bbtraj.d2theta = @(t1) (120*(t1).^3 - 180*(t1).^2 + 60*(t1));

bbtraj.T = T;
bbtraj.dT = diff(T);
bbtraj.path = path;

bbtraj.vecpos = @(t) evaluate_pos(t,bbtraj);
bbtraj.vecvel = @(t) evaluate_vel(t,bbtraj);
bbtraj.vecacc = @(t) evaluate_acc(t,bbtraj);
bbtraj.pos = @(t) single_pos(t,bbtraj);
bbtraj.vel = @(t) single_vel(t,bbtraj);
bbtraj.acc = @(t) single_acc(t,bbtraj);
bbtraj.goal = path(end,:)';



end


function [ px, py, pz ] = evaluate_pos(t,bbtraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(bbtraj.T,size(t,1),1) <= repmat(t,1,size(bbtraj.T,2)),2),[],2);
t_ind = min(t_ind,length(bbtraj.dT));

% Create the appropriate positions
t_rel = (t - bbtraj.T(:,t_ind)')./bbtraj.dT(:,t_ind)';
Pos = bsxfun(@times,bbtraj.theta(t_rel),(bbtraj.path(t_ind+1,:)-bbtraj.path(t_ind,:))) + bbtraj.path(t_ind,:);
px = Pos(:,1);
py = Pos(:,2);
pz = Pos(:,3);

end

function [ vx, vy, vz ] = evaluate_vel(t,bbtraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(bbtraj.T,size(t,1),1) <= repmat(t,1,size(bbtraj.T,2)),2),[],2);
t_ind = min(t_ind,length(bbtraj.dT));

% Create the appropriate positions using the coefficients
t_rel = (t - bbtraj.T(:,t_ind)')./bbtraj.dT(:,t_ind)';
Vel = bbtraj.dtheta(t_rel).*(bbtraj.path(t_ind+1,:)-bbtraj.path(t_ind,:));
vx = Vel(:,1);
vy = Vel(:,2);
vz = Vel(:,3);

end

function [ ax, ay, az ] = evaluate_acc(t,bbtraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(...
    repmat(bbtraj.T,size(t,1),1) <= repmat(t,1,size(bbtraj.T,2)),2),[],2);
t_ind = min(t_ind,length(bbtraj.dT));

% Create the appropriate positions using the coefficients
t_rel = (t - bbtraj.T(:,t_ind)')./bbtraj.dT(:,t_ind)';
Acc = bbtraj.d2theta(t_rel).*(bbtraj.path(t_ind+1,:)-bbtraj.path(t_ind,:));
ax = Acc(:,1);
ay = Acc(:,2);
az = Acc(:,3);

end

% Evaluate only one thing
function p = single_pos(t,bbtraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(bbtraj.T <= t, 1, 'last');
t_ind = min(t_ind,length(bbtraj.dT));

% Create the appropriate positions using path and interoplation
t_rel = (t - bbtraj.T(t_ind))/bbtraj.dT(t_ind);
p = bbtraj.theta(t_rel)*(bbtraj.path(t_ind+1,:)-bbtraj.path(t_ind,:)) + bbtraj.path(t_ind,:);
p = reshape(p,3,1);

end

function v = single_vel(t,bbtraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(bbtraj.T <= t, 1, 'last');
t_ind = min(t_ind,length(bbtraj.dT));

% Create the appropriate positions using the coefficients
t_rel = (t - bbtraj.T(t_ind))/bbtraj.dT(t_ind);
v = bbtraj.dtheta(t_rel)/bbtraj.dT(t_ind)*(bbtraj.path(t_ind+1,:)-bbtraj.path(t_ind,:));
v = reshape(v,3,1);

end

function a = single_acc(t,bbtraj)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be 1 x 1 (k being the numer of points we are evaluating)

[~, t_ind] = find(bbtraj.T <= t, 1, 'last');
t_ind = min(t_ind,length(bbtraj.dT));

% Create the appropriate positions using the coefficients
t_rel = (t - bbtraj.T(t_ind))/bbtraj.dT(t_ind);
a = bbtraj.d2theta(t_rel)/bbtraj.dT(t_ind)^2*(bbtraj.path(t_ind+1,:)-bbtraj.path(t_ind,:));
a = reshape(a,3,1);

end
