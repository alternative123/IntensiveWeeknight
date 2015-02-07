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

persistent map0 path0 spline

if nargin > 2
    map0 = map;
    path0 = path;
    % Create a smaller path
    pathopt = optimize_path(map,path);
    % Figure out times
    timeend = 14; % HOW DO WE KNOW THIS???
    pathlengths=sqrt(sum(diff(pathopt).^2,2));
    T = [0; cumsum(timeend*(pathlengths/sum(pathlengths)))]';
    
    spline = create_spline(pathopt(:,1),pathopt(:,2),pathopt(:,3),T);
    disp(spline)
    return
end

desired_state.pos = spline.pos(min(t,max(spline.T)));
desired_state.vel = spline.vel(min(t,max(spline.T)));
desired_state.acc = spline.acc(min(t,max(spline.T)));
desired_state.yaw = 0;
desired_state.yawdot = 0;

end


function pathopt = optimize_path(map,path)
pathopt = path(1:2,:);
% Get rid of unnecessary intermediate points on the path
onaline = false;
for i = 3:(size(path,1))
    % Check if we are on a line
    a = path(i,:) - pathopt(end,:);
    b = pathopt(end,:) - pathopt(end-1,:);
    if norm(a/norm(a) - b/norm(b)) < 10^-9
        pathopt(end,:) = path(i,:);
        onaline = true;
    else
        if onaline
            % Create a midpoint if two long
            if norm(b) > 5*map.xy_res
                x = pathopt(end,:);
                pathopt(end,:) = 0.5*(pathopt(end,:) + pathopt(end-1,:));
                pathopt = [ pathopt; x ];
            end
            onaline = false;
        end
        pathopt = [ pathopt; path(i,:) ];
    end
end

end

