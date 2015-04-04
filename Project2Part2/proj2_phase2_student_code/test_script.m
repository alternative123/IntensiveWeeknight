%% PREPARE DIRECTORY
close all
clc
clear
clear estimate_vel
curr_dir = pwd;
addpath(genpath([curr_dir,'/data']))

%% INITIALIZE THINGS
init_script
% load studentdata1
load studentdata4
% load studentdata9
% profile on

% Various initializations
XYZ = [-0.04, 0.0, -0.03]';
Yaw = -pi/4;
Roll = pi;
Rz = @(th) [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0, 0, 1];
Rx = @(th) [1, 0, 0; 0, cos(th), -sin(th); 0, sin(th), cos(th)];
R_toIMU = Rz(Yaw)*Rx(Roll);
T_toIMU = -R_toIMU'*XYZ;
Trans_toIMU = [R_toIMU,T_toIMU; 0 0 0 1];

% data, time, vicon
% [pos, eul] = estimate_pose(sensor, varargin)
dataL = length(data);   
ts = zeros(dataL, 1);% cumsum(ones(dataL, 1)/50);
vel = zeros(dataL, 3);
omg = zeros(dataL, 3);
pose = zeros(dataL, 3);
% eul = zeros(dataL, 3);
tic
for i = 1:dataL
    sensor = data(i);
    ts(i) = sensor.t;
    [v, o] = estimate_vel_handle(sensor);
    [p, R,~,~] = my_estimate_pose(sensor, K, tagsX, tagsY);
    if ~isempty(v)
        if i ~= 1
            vel(i, :) = v';
            omg(i, :) = o';
        end
        Trans_C = [ R, p; 0 0 0 1 ];
        Trans_B = Trans_toIMU*Trans_C;
        T = -Trans_B(1:3,1:3)'*Trans_B(1:3,4);
        pose(i, :) = T';
    end
    if mod(i,100) == 0
        fprintf('%03d...',i)
    end
    if mod(i,300) == 0
        fprintf('\n')
    end
end
fprintf('\n')
toc
time = time';
vicon = vicon';
% profile off
% profile viewer

% plot vicon data [x y z roll pitch yaw vx vy vz wx wy wz]'
% figure
% subplot(3, 1, 1)
% hold on
% plot(time, vicon(:, 1), 'r', ts, pose(:, 1), 'b')
% xlabel('Time [s]')
% ylabel('X Velocity [m/s]')
% hold off
% subplot(3, 1, 2)
% hold on
% plot(time, vicon(:, 2), 'r', ts, pose(:, 2), 'b')
% xlabel('Time [s]')
% ylabel('Y Velocity [m/s]')
% hold off
% subplot(3, 1, 3)
% hold on
% plot(time, vicon(:, 3), 'r', ts, pose(:, 3), 'b')
% xlabel('Time [s]')
% ylabel('Z Velocity [m/s]')
% hold off

% figure
% subplot(3, 1, 1)
% hold on
% plot(time, vicon(:, 4), 'r', ts, eul(:, 1), 'b')
% xlabel('Time [s]')
% ylabel('X Velocity [m/s]')
% hold off
% subplot(3, 1, 2)
% hold on
% plot(time, vicon(:, 5), 'r', ts, eul(:, 2), 'b')
% xlabel('Time [s]')
% ylabel('Y Velocity [m/s]')
% hold off
% subplot(3, 1, 3)
% hold on
% plot(time, vicon(:, 6), 'r', ts, eul(:, 3), 'b')
% xlabel('Time [s]')
% ylabel('Z Velocity [m/s]')
% hold off

% plot estimates
figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:, 7), 'r', ts, vel(:, 1), 'b')
xlabel('Time [s]')
ylabel('X Velocity [m/s]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 8), 'r', ts, vel(:, 2), 'b')
xlabel('Time [s]')
ylabel('Y Velocity [m/s]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 9), 'r', ts, vel(:, 3), 'b')
xlabel('Time [s]')
ylabel('Z Velocity [m/s]')
hold off

figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:, 10), 'r', ts, omg(:, 1), 'b')
xlabel('Time [s]')
ylabel('X Angular Velocity [rad/s]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 11), 'r', ts, omg(:, 2), 'b')
xlabel('Time [s]')
ylabel('Y Angular Velocity [rad/s]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 12), 'r', ts, omg(:, 3), 'b')
xlabel('Time [s]')
ylabel('Z Angular Velocity [rad/s]')
hold off
