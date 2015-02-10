function pathopt = optimize_path(path,for_splines)
%OPTIMIZE_PATH Summary of this function goes here
%   Detailed explanation goes here

% For later interpolation
% repmat(x,length(theta),1).*repmat(1-theta,1,size(x,2)) + repmat(y,length(theta),1).*repmat(theta,1,size(y,2))

if nargin < 2
    for_splines = false;
end

if size(path,1) < 2
    pathopt = path;
    return
end

pathopt = path(1:2,:);
% Get rid of unnecessary intermediate points on the path
linecount = 0;
maxgrouping = 60;
for i = 3:(size(path,1))
    % Check if we are on a line
    a = path(i,:) - pathopt(end,:);
    b = pathopt(end,:) - pathopt(end-1,:);
    if norm(a/norm(a) - b/norm(b)) < 10^-9 
        pathopt(end,:) = path(i,:);
        linecount = linecount + 1;
    else
        if (for_splines && linecount > maxgrouping)
            x=pathopt(end,:);
            pathopt(end,:) = 0.5*(pathopt(end,:) + pathopt(end-1,:));
            pathopt = [ pathopt; x ];
        end
        pathopt = [ pathopt; path(i,:) ];
        linecount = 0;
    end
end
assert(all(pathopt(end,:) == path(end,:)));

end