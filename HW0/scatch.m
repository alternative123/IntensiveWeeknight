% G = [
%     1.0000    2.0000    1.2000
%     2.0000    3.0000    2.4000
%     2.0000    4.0000    2.3000
%     4.0000    1.0000    1.9000
%     4.0000    3.0000    1.5000
%     4.0000    5.0000    0.6000
%     7.0000    6.0000    0.1000
%     7.0000    8.0000    5.3000 ];

n = 200; % Number of nodes
G = generateGraph(n);

start = 1;
goal = n+1;

G = sortrows(G);
mainTic = tic;

% Recreate graph in new data format
n = max(max(G(:,1)),max(G(:,2)));
infty = sum(G(:,3))*n; % large value no path could eventually weigh to
Gr = cell(n,1);

% Create the edges
for i = 1:size(G,1)
    Gr{G(i,1)} = [ Gr{G(i,1)}; G(i,2:3) ];
end
% Indecies for the graph data structure
grPath = 1;
grCost = 2;

% Create data structures
% Structure to hold all the minimum paths and their costs
minPaths = cell(n,2);
% What nodes have we seen, or found the min path for?
seen = zeros(n,1);
explored = zeros(n,1)==1;
% Frontier of points we are looking at
% Indecies of these data structures for determining what goes where
pathInd = 1;
costInd = 2;
frontier = [ (1:n)' ones(n,2)*infty ];
frontier(start, pathInd) = start;
frontier(start, costInd) = 0;
seen(start) = 1; % Have seen the start node

t = 1;
% Begin Djikstras
while true % Begin loop
    % Get min index
    [~,minNode] = min((infty*explored)+frontier(:,costInd));
    minPath = frontier(minNode,pathInd);
    minCost = frontier(minNode,costInd);
    % Short cut the algorithm if we found our goal
    if minNode == goal
        path = [ minNode; minPaths{minPath,pathInd} ];
        cost = minCost;
        break % return
    end
    % Otherwise set up for the next iteration
    % Set node to 'explored'
    explored(minNode) = true;
    % Set the path value
    minPaths{minNode,pathInd} = [ minPaths{minPath,pathInd} ; minNode ];
    % Set the cost value
    minPaths{minNode,costInd} = minCost;
    % Update things
    for i = 1:size(Gr{minNode},1)
        node = Gr{minNode}(i,grPath);
        altCost = minCost + Gr{minNode}(i,grCost);
        if altCost < frontier(node,costInd)
            frontier(node,costInd) = altCost;
            frontier(node,pathInd) = minNode;
            seen(node) = 1;
        end
    end
    
    t = t + 1;
    
    if all((seen - explored) == 0)
        break
    end
end


totalTime = toc(mainTic);
%%





