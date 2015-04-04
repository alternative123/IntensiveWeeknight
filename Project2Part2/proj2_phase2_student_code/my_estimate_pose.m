function [T, R, p_i, p_w] = my_estimate_pose(sensor, K, tagsX, tagsY)
% Check if we see any tags:
if any(size(sensor.id) == 0)
    T = [];
    R = [];
    return
end

% Camera-IMU Calibration (see attached images for details):
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

R_cw = U*[1,0,0; 0,1,0; 0,0,det(U*V')]*V';
T_cw = H(:,3) / norm(H(:,1));
R = R_cw;
T = T_cw;

% Trans_C = [ R1, T1; 0 0 0 1 ];
% Trans_B = Trans_toIMU*Trans_C;
% T = -Trans_B(1:3,1:3)'*Trans_B(1:3,4);
% R = Trans_B(1:3,1:3);

end