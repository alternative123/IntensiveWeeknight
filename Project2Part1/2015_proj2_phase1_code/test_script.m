%% PREPARE DIRECTORY
close all
clc
curr_dir = pwd;
addpath(genpath([curr_dir,'/data']))

%% INITIALIZE THINGS
init_script
load studentdata1
% load studentdata4
% load studentdata9


% data, time, vicon
% [pos, eul] = estimate_pose(sensor, varargin)
dataL = length(data);
ts = zeros(dataL, 1);
pose = zeros(dataL, 3);
eul = zeros(dataL, 3);
for i = 1:dataL
    sensor = data(i);
    ts(i) = sensor.t;
    if i == dataL/2
        disp('hello!!')
    end
    [p, e] = estimate_pose_handle(sensor);
    if ~isempty(p)
        pose(i, :) = p';
        eul(i, :) = e';
    end
end

time = time';
vicon = vicon';

% plot vicon data [x y z roll pitch yaw vx vy vz wx wy wz]'
% plot estimates
figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:, 1), 'r', ts, pose(:, 1), 'b')
xlabel('Time [s]')
ylabel('X Position [m]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 2), 'r', ts, pose(:, 2), 'b')
xlabel('Time [s]')
ylabel('Y Position [m]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 3), 'r', ts, pose(:, 3), 'b')
xlabel('Time [s]')
ylabel('Z Position [m]')
hold off

figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:, 4), 'r', ts, eul(:, 1), 'b')
xlabel('Time [s]')
ylabel('Roll [rad]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 5), 'r', ts, eul(:, 2), 'b')
xlabel('Time [s]')
ylabel('Pitch [rad]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 6), 'r', ts, eul(:, 3), 'b')
xlabel('Time [s]')
ylabel('Yaw [rad]')
hold off
