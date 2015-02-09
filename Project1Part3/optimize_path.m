function pathopt = optimize_path(path,map)
%OPTIMIZE_PATH Summary of this function goes here
%   Detailed explanation goes here


if size(path,1) < 2
    pathopt = path;
    return
end

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
            if nargin > 1 && norm(b) > 5*map.xy_res
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