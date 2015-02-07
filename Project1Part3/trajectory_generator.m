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

persistent map0 path0 pathopt

if nargin > 2
    if isempty(map0)
        map0 = map;
    end
    if isempty(path0)
        path0 = path;
        % Create a smaller path
        pathopt = optimize_path(path);
    end
    % Figure out times
    
    return
end

end


function pathopt = optimize_path(path)
pathopt = path(1:2,:);
% Get rid of unnecessary intermediate points on the path
for i = 3:(size(path,1))
    % Check if we are on a line
    a = path(i,:) - pathopt(end,:);
    b = pathopt(end,:) - pathopt(end-1,:);
%     fprintf('a=(%f,%f,%f); b=(%f,%f,%f);\n',a(1),a(2),a(3),b(1),b(2),b(3));
%     fprintf('path(i)=(%f,%f,%f); pathopt(end)=(%f,%f,%f); pathopt(end-1)=(%f,%f,%f)\n',...
%         path(i,1),path(i,2),path(i,3),...
%         pathopt(end,1),pathopt(end,2),pathopt(end,3),...
%         pathopt(end-1,1),pathopt(end-1,2),pathopt(end-1,3));
%     fprintf('Pass? %d ((%f,%f,%f) vs (%f,%f,%f)) \n',...
%         all(a/norm(a) == b/norm(b)),...
%         a(1)/norm(a),a(2)/norm(a),a(3)/norm(a),...
%         b(1)/norm(b),b(2)/norm(b),b(3)/norm(b));
    if norm(a/norm(a) - b/norm(b)) < 10^-9
%         fprintf('here %d\n\n\n',i)
        pathopt(end,:) = path(i,:);
    else
%         fprintf('there %d\n\n\n',i)
        pathopt = [ pathopt; path(i,:) ];
    end
end

end

