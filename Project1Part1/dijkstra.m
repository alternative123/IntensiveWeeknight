function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end

% With the size of these matrices, we'll need to redesign Dijkstras from
% HW1 - even storing all the nodes is too much. (Is it though?)

n = prod(map.gridsize);
nx = map.gridsize(1);
ny = map.gridsize(2);
nz = map.gridsize(3);

% Offsets to get to center of grid
xoffset = map.xy_res/2;
yoffset = map.xy_res/2;
zoffset = map.z_res/2;

% Helper functions for converting between indeces and 3D coordinates
xindToX = @(x) x*map.xy_res + map.boundary(1,1) - xoffset;
yindToY = @(y) y*map.xy_res + map.boundary(1,2) - yoffset;
zindToZ = @(z) z*map.z_res + map.boundary(1,3) - zoffset;

xToXind = @(x) ceil((x - map.boundary(1,1))/map.xy_res+eps);
yToYind = @(y) ceil((y - map.boundary(1,2))/map.xy_res+eps);
zToZind = @(z) ceil((z - map.boundary(1,3))/map.z_res+eps);
xyzToInd = @(x,y,z) xToXind(x) + nx*(yToYind(y)-1) + nx*ny*(zToZind(z)-1);

% Generate collision table
freespace = true(map.gridsize);
tic
[x,y] = meshgrid(1:map.gridsize(1), 1:map.gridsize(2));
for o = 1:size(map.obstacles,1)
    box = map.obstacles(o,:);
    for z = 1:map.gridsize(3)
        % Calculate 3d positions of the indeces
        p = [...
            xindToX(x(:)),...
            yindToY(y(:)),...
            zindToZ(z*ones(numel(x),1)) ...
            ];
        % Calculate distance from obstacle o
        dx = max(max(box(1)-p(:,1),p(:,1)-box(4)),zeros(size(p,1),1));
        dy = max(max(box(2)-p(:,2),p(:,2)-box(5)),zeros(size(p,1),1));
        dz = max(max(box(3)-p(:,3),p(:,3)-box(6)),zeros(size(p,1),1));
        freepoints = ((dx.^2 + dy.^2 + dz.^2) > map.margin^2);
        freespace(:,:,z) = freespace(:,:,z) & ...
            reshape(freepoints,map.gridsize([2 1]))';
    end
end
toc
tic
% Precompute local edge weights
% allcomb = combvec(-1:1,-1:1,-1:1)'; % 26-connected
allcomb = [ 1  0  0;
           -1  0  0;
            0  1  0;
            0 -1  0;
            0  0 -1;
            0  0  1 ]; % 6-connected
cx = allcomb(:,1); % cx for 'combinations of -1,0,1 for x'.
cy = allcomb(:,2); % Similar for these two
cz = allcomb(:,3);
localWeights = sqrt((map.xy_res*cx).^2+(map.xy_res*cy).^2+(map.z_res*cz).^2);

% Find appropriate starting point
startXYZ = [xindToX(xToXind(start(1))) yindToY(yToYind(start(2))) zindToZ(zToZind(start(3)))];
goalXYZ = [xindToX(xToXind(goal(1))) yindToY(yToYind(goal(2))) zindToZ(zToZind(goal(3)))];
startNode = xyzToInd(start(1),start(2),start(3));
goalNode = xyzToInd(goal(1),goal(2),goal(3));

% Create data structures
% Initialize frontier/seen data structures
paths = ones(map.gridsize)*Inf;
paths(startNode) = -1;

% Actual cost vector
cost = ones(map.gridsize)*Inf;
cost(startNode) = 0;

% Heuristic vector for astar if needed
fscore = ones(map.gridsize)*Inf;
if astar
    fscore(startNode) = 0 + norm(startXYZ-goalXYZ);
else
    fscore(startNode) = 0;
end

t = 0;
toc
tic
% Begin Djikstras
while true % Begin loop
    % Get min node and cost
    [minFscore,minNode] = min(fscore(:));
    minScore = cost(minNode);
    if minFscore == Inf
        path = zeros(0,1);
        num_expanded = t + 1;
        return
    end
    % Short cut the algorithm if we found our goal
    if minNode == goalNode
        path = createPath(map,paths,goal,start,goalNode,startNode);%[ minPaths{minPath,pathInd} ; minNode ];
        num_expanded = t + 1;
        return
    end
    t = t + 1;

    % Otherwise set up for the next iteration
    % Set node to 'explored'
    freespace(minNode) = false;
    cost(minNode) = Inf;
    fscore(minNode) = Inf;

    % Update things - 26 connected nodes
    % Get the proper indeces
    [xi,yi,zi] = ind2sub(map.gridsize,minNode);
    indeces = (xi+cx) + nx*(yi+cy-1) + nx*ny*(zi+cz-1);
    % Find valid indeces -> inside the boundary and free space
    validindeces = (xi+cx > 0 & yi+cy > 0 & zi+cz > 0 & xi+cx <= nx & yi+cy <= ny & zi+cz <= nz);
    validindeces(validindeces) = freespace(indeces(validindeces));

    if all(~validindeces)
        continue
    end

    % Update the costs/fscores
    newCosts = minScore + localWeights(validindeces);
    cost(indeces(validindeces)) = min(newCosts, cost(indeces(validindeces)));
    if astar
        heuristic = sqrt(...
            (goalXYZ(1)-xindToX(xi+cx(validindeces))).^2 + ...
            (goalXYZ(2)-yindToY(yi+cy(validindeces))).^2 + ...
            (goalXYZ(3)-zindToZ(zi+cz(validindeces))).^2);
        fscore(indeces(validindeces)) = cost(indeces(validindeces)) + heuristic;
    else
        fscore(indeces(validindeces)) = min(newCosts, cost(indeces(validindeces)));
    end
    % Update the paths
    updatedCosts = min(newCosts, cost(indeces(validindeces))) == newCosts;
    updatePaths = indeces(validindeces);
    updatePaths = updatePaths(updatedCosts);
    paths(updatePaths) = minNode;
    %if mod(t,1000) == 0
    %    fprintf('%d with %f %% done... \n',t, 1-(sum(sum(sum(freespace)))/numel(freespace)));
    %end
end
toc
% If we get here something went seriously wrong...

path = [];
num_expanded = 0;
end



function finalpath = createPath(map,paths,goalXYZ,startXYZ,goalNode,startNode)
% Helper function
xyzIndToXYZ = @(x,y,z) [ x*map.xy_res + map.boundary(1,1) - map.xy_res/2;
                         y*map.xy_res + map.boundary(1,2) - map.xy_res/2;
                         z*map.z_res + map.boundary(1,3) - map.z_res/2;  ];
% Start path at the end and work back
p = goalNode;
X = goalXYZ;
[xi,yi,zi] = ind2sub(map.gridsize,p(end));
X = [ X; xyzIndToXYZ(xi,yi,zi)' ]; % Now we start in the grid

while p(end) ~= startNode
    p = [p; paths(p(end))];
    [xi,yi,zi] = ind2sub(map.gridsize,p(end));
    X = [ X; xyzIndToXYZ(xi,yi,zi)' ];
end

% Append start node to true start coordinates
finalpath = flipud([X;startXYZ]);

end
