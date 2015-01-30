% G = [
%     1.0000    2.0000    1.2000
%     2.0000    3.0000    2.4000
%     4.0000    3.0000    1.5000
%     4.0000    5.0000    0.6000
%     4.0000    1.0000    1.9000
%     2.0000    4.0000    2.3000
%     7.0000    8.0000    5.3000
%     7.0000    6.0000    0.1000 ];

n = 200;
G = generateGraph(n);

start = 1;
goal = n+1;

%G = sortrows(G);
mainTic = tic;

% Recreate graph in new data format
Gr = cell(max(max(G(:,1)),max(G(:,2))),1);

% Create the edges
for i = 1:size(G,1)
    Gr{G(i,1)} = [ Gr{G(i,1)}; G(i,2:3) ];
end
% Indecies for the graph data structure
grPath = 1;
grCost = 2;

% Create data structures
% Structure to hold all the minimum paths and their costs
minPaths = cell(size(Gr,1),3);
% Frontier of points we are looking at
frontier = [ start start 0 ];
% Indecies of these data structures for determining what goes where
nodeInd = 1;
pathInd = 2;
costInd = 3;
% Nodes explored
explored = [];

timeGraph = zeros(1,1000);
time1Graph = zeros(1,1000);
time2Graph = zeros(1,1000);
time3Graph = zeros(1,1000);
time4Graph = zeros(1,1000);

t = 1;
% Begin Djikstras
while size(frontier,1) > 0 % while we haven't seen the goal yet
    loopTime = tic;
    % % Grab the top of the frontier, and append it to our minPaths
    % Find minimum cost on the frontier
    [~,minInd] = min(frontier(:,costInd));
    minNode = frontier(minInd,nodeInd);
    minPath = frontier(minInd,pathInd);
    minCost = frontier(minInd,costInd);
    time1Graph(t) = toc(loopTime);
    
    % Short cut the algorithm if we found our goal
    if minNode == goal
        path = [ minNode; minPaths{minPath,pathInd} ];
        cost = minCost;
    else
        % Set the node value
        minPaths{minNode,nodeInd} = minNode;

        % Set the path value
        minPaths{minNode,pathInd} = [ minNode; minPaths{minPath,pathInd} ];

        % Set the cost value
        minPaths{minNode,costInd} = minCost;
    end
    time2Graph(t) = toc(loopTime);
    

    % Add all paths going out from the node we just added to our frontier
    for i = 1:size(Gr{minNode},1)
        frontier = [ 
                frontier ;
                [
                    Gr{minNode}(i,grPath), ...
                    minNode, ...
                    (minCost+Gr{minNode}(i,grCost))
                ]
            ];
    end
    time3Graph(t) = toc(loopTime);

    % Remove that node from that node from the frontier
    explored = [ explored; minNode ];
    for e = explored'
        frontier(frontier(:,nodeInd)==e,:) = [];
    end
    timeGraph(t) = toc(loopTime);
    t = t + 1;
end


totalTime = toc(mainTic);
disp(totalTime)
niters = find(timeGraph==0,1);

timeGraph = timeGraph(1:niters);
time1Graph = time1Graph(1:niters);
time2Graph = time2Graph(1:niters);
time3Graph = time3Graph(1:niters);
time4Graph = time4Graph(1:niters);

plot(timeGraph,'k');
hold on
plot(time1Graph,'r');
plot(time2Graph,'g');
plot(time3Graph,'b');
% plot(time4Graph,'b');

hold off

