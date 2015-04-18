function [pos, eul, R, T] = estimate_pose(sensor, K,tagsX,tagsY)
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


% Check if we see any tags:
if any(size(sensor.id) == 0)
    pos = [];
    eul = [];
    R = [];
    T = [];
    return
end

% Camera-IMU Calibration (see attached images for details):
XYZ = [-0.04, 0.0, -0.03]';
Yaw = -pi/4;
Roll = pi;
Rz = @(th) [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0, 0, 1];
Rx = @(th) [1, 0, 0; 0, cos(th), -sin(th); 0, sin(th), cos(th)];
R_toIMU = Rz(Yaw)*Rx(Roll);
T_toIMU = -R_toIMU'*XYZ;
Trans_toIMU = [ R_toIMU', T_toIMU; 0 0 0 1 ];
w_tag = 0.152; % Tag width

ids = sensor.id+1; % April tag ids seen in this image
% Create the points needed for the Homography estimation
p_w = ...
    [ tagsX(ids), tagsX(ids)      , tagsX(ids)+w_tag, tagsX(ids)+w_tag, tagsX(ids)+w_tag/2;
      tagsY(ids), tagsY(ids)+w_tag, tagsY(ids)      , tagsY(ids)+w_tag, tagsY(ids)+w_tag/2;
      ones(1,5*length(ids))  ];
p_i = [ sensor.p4, sensor.p3, sensor.p1, sensor.p2, sensor.p0;
       ones(1,5*length(ids)) ];

% Allocate the matrix size
A = zeros(2*length(p_w), 9);

% Instantiate the matrix
for i = 1:length(p_w)
   row = 2*(i-1) + 1;
   % Set up the two rows associated with this point relation
   A(row, :)     = [p_w(:,i)', zeros(1,3), -p_i(1,i)*p_w(:,i)'];
   A(row + 1, :) = [zeros(1,3), p_w(:,i)', -p_i(2,i)*p_w(:,i)'];
end

% Find the kernel of this matrix using SVD
[~, ~, V] = svd(A);
x = V(:, end);    % The last column of V is its kernel

% Return the H matrix
H = reshape(x,3,3)';
H = H/H(3,3);
H = K\H;

% Transform solution into desired frame
[U,~,V] = svd([H(:,1) H(:,2) cross(H(:,1),H(:,2))]);

R = U*[1,0,0; 0,1,0; 0,0,det(U*V')]*V';
T = H(:,3) / norm(H(:,1));

Trans_C = [ R, T; 0 0 0 1 ];
Trans_B = Trans_toIMU*Trans_C;
pos = -Trans_B(1:3,1:3)'*Trans_B(1:3,4);

phi = asin(Trans_B(2,3));
psi = atan2(-Trans_B(2,1)/cos(phi),Trans_B(2,2)/cos(phi));
theta = atan2(-Trans_B(1,3)/cos(phi),Trans_B(3,3)/cos(phi));
eul = [phi,theta,psi]';


end
