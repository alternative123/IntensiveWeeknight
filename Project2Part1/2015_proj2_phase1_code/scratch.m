load('/home/stephen/Dropbox/Documents/PhD/Classes/MEAM620/Project2Part1/2015_proj2_phase1_code/data/studentdata1.mat')
sensor = data(1);
%%
% Camera Matrix (zero-indexed):
K = [314.1779 0         199.4848; ...
     0         314.2218  113.7838; ...
     0         0         1];

% Camera-IMU Calibration (see attached images for details):
XYZ = [-0.04, 0.0, -0.03];
Yaw = pi/4;

% Compute the positions of the April tags
% Tag ids:
w_tag = 0.152; % Tag width
tags = [  0, 12, 24, 36, 48, 60, 72, 84,  96;
          1, 13, 25, 37, 49, 61, 73, 85,  97;
          2, 14, 26, 38, 50, 62, 74, 86,  98;
          3, 15, 27, 39, 51, 63, 75, 87,  99;
          4, 16, 28, 40, 52, 64, 76, 88, 100;
          5, 17, 29, 41, 53, 65, 77, 89, 101;
          6, 18, 30, 42, 54, 66, 78, 90, 102;
          7, 19, 31, 43, 55, 67, 79, 91, 103;
          8, 20, 32, 44, 56, 68, 80, 92, 104;
          9, 21, 33, 45, 57, 69, 81, 93, 105;
         10, 22, 34, 46, 58, 70, 82, 94, 106;
         11, 23, 35, 47, 59, 71, 83, 95, 107  ];

tagsX = (2*w_tag)*ones(size(tags));
tagsX(1,:) = 0; 
tagsX = cumsum(tagsX);

tagsY = (2*w_tag)*ones(size(tags));
tagsY(:,1) = 0; 
tagsY(:,4) = w_tag+0.178; % Because the April tag folks decided that it 
tagsY(:,7) = w_tag+0.178; % would be better to make things wierd for us 
tagsY = cumsum(tagsY,2);


ids = sensor.id+1; % April tag ids seen in this image
% Create the points
p1 = ...
    [ tagsX(ids), tagsX(ids)      , tagsX(ids)+w_tag, tagsX(ids)+w_tag, tagsX(ids)+w_tag/2;
      tagsY(ids), tagsY(ids)+w_tag, tagsY(ids)      , tagsY(ids)+w_tag, tagsY(ids)+w_tag/2;
      ones(1,5*length(ids))  ];
p2 = [ sensor.p4, sensor.p3, sensor.p1, sensor.p2, sensor.p0;
       ones(1,5*length(ids)) ];

H = solveProjective(p1,p2);

%% Round 2

% Compute the positions of the April tags
% Tag ids:
w_tag = 0.152; % Tag width
tags = [  0, 12, 24, 36, 48, 60, 72, 84,  96;
          1, 13, 25, 37, 49, 61, 73, 85,  97;
          2, 14, 26, 38, 50, 62, 74, 86,  98;
          3, 15, 27, 39, 51, 63, 75, 87,  99;
          4, 16, 28, 40, 52, 64, 76, 88, 100;
          5, 17, 29, 41, 53, 65, 77, 89, 101;
          6, 18, 30, 42, 54, 66, 78, 90, 102;
          7, 19, 31, 43, 55, 67, 79, 91, 103;
          8, 20, 32, 44, 56, 68, 80, 92, 104;
          9, 21, 33, 45, 57, 69, 81, 93, 105;
         10, 22, 34, 46, 58, 70, 82, 94, 106;
         11, 23, 35, 47, 59, 71, 83, 95, 107  ];

tagsX = (2*w_tag)*ones(size(tags));
tagsX(1,:) = 0; 
tagsX = cumsum(tagsX);

tagsY = (2*w_tag)*ones(size(tags));
tagsY(:,1) = 0; 
tagsY(:,4) = w_tag+0.178; % Because the April tag folks decided that it 
tagsY(:,7) = w_tag+0.178; % would be better to make things wierd for us 
tagsY = cumsum(tagsY,2);


%%
ids = sensor.id+1; % April tag ids seen in this image
% Create the points needed for the Homography estimation
p1 = ...
    [ tagsX(ids), tagsX(ids)      , tagsX(ids)+w_tag, tagsX(ids)+w_tag, tagsX(ids)+w_tag/2;
      tagsY(ids), tagsY(ids)+w_tag, tagsY(ids)      , tagsY(ids)+w_tag, tagsY(ids)+w_tag/2;
      ones(1,5*length(ids))  ];
p2 = [ sensor.p4, sensor.p3, sensor.p1, sensor.p2, sensor.p0;
       ones(1,5*length(ids)) ];

% get number of points
[~, npoints] = size(p1);

% Allocate the matrix size
A = zeros(2*npoints, 9);

% Instantiate the matrix
for i = 1:npoints
   row = 2*(i-1) + 1;
   % Set up the two rows associated with this point relation
   A(row, :)     = [p1(:,i)', zeros(1,3), -p2(1,i)*p1(:,i)'];
   A(row + 1, :) = [zeros(1,3), p1(:,i)', -p2(2,i)*p1(:,i)'];
end
% Find the kernel of this matrix using SVD
[~, ~, V] = svd(A);
x = V(:, end);    % The last column of V is its kernel

% Return the H matrix
H = reshape(x,3,3)' / x(9);



%% Debugging visualizations
figure
for i = 1:length(tagsX(:))
    scatter(...
        [tagsX(i) 
         tagsX(i) + w_tag
         tagsX(i)
         tagsX(i) + w_tag
         tagsX(i) + w_tag/2],...
        [tagsY(i) 
         tagsY(i)
         tagsY(i) + w_tag
         tagsY(i) + w_tag
         tagsY(i) + w_tag/2])
    hold on
end
legendCell = cellstr(num2str((1:24)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal
%%

figure
for i = 1:length(p1)/5
    scatter(...
        [p1(1,i)./p1(3,i), 
         p1(1,i+length(ids))./p1(3,i+length(ids))
         p1(1,i+2*length(ids))./p1(3,i+2*length(ids))
         p1(1,i+3*length(ids))./p1(3,i+3*length(ids))
         p1(1,i+4*length(ids))./p1(3,i+4*length(ids))],...
        [p1(2,i)./p1(3,i), 
         p1(2,i+length(ids))./p1(3,i+length(ids))
         p1(2,i+2*length(ids))./p1(3,i+2*length(ids))
         p1(2,i+3*length(ids))./p1(3,i+3*length(ids))
         p1(2,i+4*length(ids))./p1(3,i+4*length(ids))])
    hold on
end
legendCell = cellstr(num2str((ids)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal
figure
for i = 1:length(p1)/5
    scatter(...
        [p2(1,i)./p1(3,i) 
         p2(1,i+length(ids))./p1(3,i+length(ids))
         p2(1,i+2*length(ids))./p1(3,i+2*length(ids))
         p2(1,i+3*length(ids))./p1(3,i+3*length(ids))
         p2(1,i+4*length(ids))./p1(3,i+4*length(ids))],...
        [p2(2,i)./p1(3,i) 
         p2(2,i+length(ids))./p1(3,i+length(ids))
         p2(2,i+2*length(ids))./p1(3,i+2*length(ids))
         p2(2,i+3*length(ids))./p1(3,i+3*length(ids))
         p2(2,i+4*length(ids))./p1(3,i+4*length(ids))])
    hold on
end
legendCell = cellstr(num2str((ids)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal

q1 = H*p1;
q1(1,:) = q1(1,:)./q1(3,:);
q1(2,:) = q1(2,:)./q1(3,:);
figure
for i = 1:length(p1)/5
    scatter(...
        [q1(1,i)./p1(3,i) 
         q1(1,i+length(ids))./p1(3,i+length(ids))
         q1(1,i+2*length(ids))./p1(3,i+2*length(ids))
         q1(1,i+3*length(ids))./p1(3,i+3*length(ids))
         q1(1,i+4*length(ids))./p1(3,i+4*length(ids))],...
        [q1(2,i)./p1(3,i), 
         q1(2,i+length(ids))./p1(3,i+length(ids))
         q1(2,i+2*length(ids))./p1(3,i+2*length(ids))
         q1(2,i+3*length(ids))./p1(3,i+3*length(ids))
         q1(2,i+4*length(ids))./p1(3,i+4*length(ids))])
    hold on
end
legendCell = cellstr(num2str((ids)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal

for i = 1:length(p1)

end

hat = @(x) [ 0,-x(3),x(2); x(3),0,-x(1); -x(2),x(1),0 ];

%% Testing again...











