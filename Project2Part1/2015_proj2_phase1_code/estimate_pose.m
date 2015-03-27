function [pos, eul] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   eul - 3x1 euler angles of the quadrotor

% Camera Matrix (zero-indexed):
K = [314.1779 0         199.4848; ...
     0        314.2218  113.7838; ...
     0        0         1          ];

% Camera-IMU Calibration (see attached images for details):
XYZ = [-0.04, 0.0, -0.03]';
Yaw = pi/4;
Roll = pi;
Rz = @(th) [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0, 0, 1];
Rx = @(th) [0, 0, 1; 0, cos(th), -sin(th); 0, sin(th), cos(th)];
R_toIMU = Rz(Yaw)*Rx(Roll);
T_toIMU = -R_toIMU'*XYZ;
Trans_toIMU = [ R_toIMU, T_toIMU; 0 0 0 1 ];

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
x = V(:, 9);    % The last column of V is its kernel

% Return the H matrix
H = reshape(x,3,3)' / x(9);

H = K\H;

% Transform solution into desired frame
[U,~,V] = svd([H(:,1) H(:,2) cross(H(:,1),H(:,2))]);

R1 = U*[1,0,0;0,1,0;0,0,det(U*V')]*V';
T1 = H(:,3) / norm(H(:,1));

Trans_C = [ R1, T1; 0 0 0 1 ];
Trans_W = Trans_C*Trans_toIMU;
pos = -Trans_W*[Trans_W(1:3,4), 0];
pos = pos(1:3);

eul = zero(3,1);
eul(1) = atan2(Trans_W(3,2),Trans_W(2,3));
eul(2) = atan2(-Trans_W(3,1),norm([Trans_W(3,2),Trans_W(3,3)]));
eul(3) = atan2(Trans_W(2,1),Trans_W(1,1));

end
