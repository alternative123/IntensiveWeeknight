function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

persistent traj

if nargin > 2
    % Create a smaller path
    if iscell(path)
        path2 = cell2mat(path);%optimize_path(map,path);
    else
        path2 = path;
    end
   
    traj = create_spline(path2,map);
    return
end

if t < max(traj.T)
    desired_state.pos = traj.pos(t);
    desired_state.vel = traj.vel(t);
    desired_state.acc = traj.acc(t);
else
    desired_state.pos = traj.goal;
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
end

desired_state.yaw = 0;
desired_state.yawdot = 0;

end



