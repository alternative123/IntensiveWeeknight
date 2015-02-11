function pathopt = optimize_path(path,map, for_splines)
%OPTIMIZE_PATH Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    for_splines = false;
end

if size(path,1) < 2
    pathopt = path;
    return
end

pathopt = path(1:2,:);
% Get rid of unnecessary intermediate points on the path
groupcount = 0;
maxgrouping = 30;
for i = 3:(size(path,1))
    % Check if we are on a line
    a = path(i,:) - pathopt(end,:);
    b = pathopt(end,:) - pathopt(end-1,:);
    if isfree(pathopt(end-1,:),path(i,:),map) && groupcount < maxgrouping % norm(a/norm(a) - b/norm(b)) < 10^-9 
        pathopt(end,:) = path(i,:);
        groupcount = groupcount + 1;
    else
        if (for_splines && groupcount > maxgrouping)
            x=pathopt(end,:);
            pathopt(end,:) = 0.5*(pathopt(end,:) + pathopt(end-1,:));
            pathopt = [ pathopt; x ];
        end
        pathopt = [ pathopt; path(i,:) ];
        groupcount = 0;
    end
end
assert(all(pathopt(end,:) == path(end,:)));
end

% This is an old but still pretty good way to do it
% for i = 3:(size(path,1))
%     Check if we are on a line
%     a = path(i,:) - pathopt(end,:);
%     b = pathopt(end,:) - pathopt(end-1,:);
%     if isfree(pathopt(end-1,:),path(i,:),map) % norm(a/norm(a) - b/norm(b)) < 10^-9 
%         pathopt(end,:) = path(i,:);
%         groupcount = groupcount + 1;
%     else
%         if (for_splines && groupcount > maxgrouping)
%             x=pathopt(end,:);
%             pathopt(end,:) = 0.5*(pathopt(end,:) + pathopt(end-1,:));
%             pathopt = [ pathopt; x ];
%         end
%         pathopt = [ pathopt; path(i,:) ];
%         groupcount = 0;
%     end
% end


function free = isfree(x,y,map)

theta = linspace(0,1,10*norm(x-y)/min(map.xy_res,map.z_res))';
pts = repmat(x,length(theta),1).*repmat(1-theta,1,size(x,2)) + repmat(y,length(theta),1).*repmat(theta,1,size(y,2));

% if norm(x-y) > 12
%     disp(sum(collide(map,pts)))
%     disp(100*norm(x-y)/min(map.xy_res,map.z_res))
%     disp('Large displacement')
%     plot_path(map,pts)
% end

free = ~any(collide(map,pts));

end
