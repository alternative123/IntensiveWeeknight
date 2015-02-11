% Figuring out splines

% T = [ 0 1 2 4 5 6    8 9];
X = [ 0,1,2,3,4, 5, 4, 3, 2, 1, 0,-1,-2,-3,-4,-5,-4,-3,-2,-1,0 ];
Y = [ 5,6,7,8,9,10,11,12,13,14,15,14,13,12,11,10, 9, 8, 7, 6,5 ];
Z = Y;
T = 0:length(X)-1;

path = [ X(1)-0.5 Y(1) Z(1); X(:) Y(:) Z(:) ];


% Optimize path
% If the path is too short, don't optimize it
%
clear pathopt
pathopt = optimize_path(path);

scatter(pathopt(:,1),pathopt(:,2),30*(1:size(pathopt,1)),[1 0 0])

% Figure out times automatically
timeend = 14;
pathlengths=sqrt(sum(diff(pathopt).^2,2));
T = [0; cumsum(timeend*(pathlengths/sum(pathlengths)))]';

%

% Spline it up
% spline = create_spline(pathopt(:,1),pathopt(:,2),pathopt(:,3),T);
bbtraj = create_bangbang_trajectory(pathopt,T);

[x,y,z] = bbtraj.vecpos((0:0.1:max(T))');
scatter(X,Y)
hold on
plot(x,y)















