close all;
clear;
clc;

initialize;

%% Simulate the system using ODE45

[Ts, XS] = ode45(@(t,x) noisy_model(t, x, A, B, sgm, bias), Ts, x0); % Solve ODE
XS = XS';



%% Simualate the sensors

for i=1:4 
XSMax(i)=max(XS(i,:));
end
ZS(1,:)=XS(1,:)+0.05*randn(size(Ts'))*XSMax(1);
ZS(2,:)=XS(2,:)+0.1*randn(size(Ts'))*XSMax(2);
ZS(3,:)=XS(3,:)+0.15*randn(size(Ts'))*XSMax(3);
ZS(4,:)=XS(4,:)+0.2*randn(size(Ts'))*XSMax(4);

%% Kalman Filter - part a

% Our estimations variables
mu = zeros(N,length(Ts));
Sigma = zeros(N,N,length(Ts));
Sigma(:,:,1) = P0;

% H is the measurement matrix
H = eye(N);
% F,G are the linear transition model. 
% Q,R are the noise covariance matrices.
R  = 10*eye(N) * sgm^2;
V = dt;

for i = 2:length(Ts)
    % Prediction step
    mu_prev = F*mu(:,i-1) + G*input_fun(Ts(i));
    Sigma_prev = F*Sigma(:,:,i-1)*F' + V*Q*V';
    
    % Update step
    K = (Sigma_prev*H')/(H*Sigma_prev*H' + R); % Kalman gain
    mu(:,i) = mu_prev + K*(ZS(:,i) - H*mu_prev);
    Sigma(:,:,i) = Sigma_prev - K*H*Sigma_prev;
end

% Plot our estimates

figure; hold on; grid on;
plot(Ts, ZS(1, :), 'r');
plot(Ts, XS(1, :), 'k'); 
plot(Ts, mu(1, :), 'c'); 
h = legend('$z$','$x$','$\hat{x}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Position)');

figure; hold on; grid on;
plot(Ts, ZS(2, :), 'b');
plot(Ts, XS(2, :), 'k');
plot(Ts, mu(2, :), 'c'); 
h = legend('$\dot{z}$','$\dot{x}$','$\dot{\hat{x}}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Velocity)');

figure; hold on; grid on;
plot(Ts, ZS(3, :), 'g');
plot(Ts, XS(3, :), 'k');
plot(Ts, mu(3, :), 'c'); 
h = legend('$\ddot{z}$','$\ddot{x}$','$\ddot{\hat{x}}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Acceleration)');

figure; hold on; grid on;
plot(Ts, ZS(4, :), 'r');
plot(Ts, XS(4, :), 'k')
plot(Ts, mu(4, :), 'c'); 
h = legend('${z}^{(3)}$','${x}^{(3)}$','$\hat{x}^{(3)}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Jerk)');



%% Kalman Filter - with reduced input - part b

% Our estimations variables
mu = zeros(N,length(Ts));
Sigma = zeros(N,N,length(Ts));
Sigma(:,:,1) = P0;

% H is the measurement matrix
H = [1,0,0,0];
% F,G are the linear transition model. 
% Q,R are the noise covariance matrices.
R = 10*sgm^2;
z = ZS(1,:);
V = dt;


for i = 2:length(Ts)
    % Prediction step
    mu_prev = F*mu(:,i-1) + G*input_fun(Ts(i));
    Sigma_prev = F*Sigma(:,:,i-1)*F' + V*Q*V';
    
    % Update step
    K = (Sigma_prev*H')/(H*Sigma_prev*H' + R); % Kalman gain
    mu(:,i) = mu_prev + K*(z(i) - H*mu_prev);
    Sigma(:,:,i) = Sigma_prev - K*H*Sigma_prev;
end

% Plot our estimates

figure; hold on; grid on;
plot(Ts, ZS(1, :), 'r');
plot(Ts, XS(1, :), 'k'); 
plot(Ts, mu(1, :), 'c'); 
h = legend('$z$','$x$','$\hat{x}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Position');

figure; hold on; grid on;
plot(Ts, XS(2, :), 'k');
plot(Ts, mu(2, :), 'c'); 
h = legend('$\dot{z}$','$\dot{x}$','$\dot{\hat{x}}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Velocity');

figure; hold on; grid on;
plot(Ts, XS(3, :), 'k');
plot(Ts, mu(3, :), 'c'); 
h = legend('$\ddot{z}$','$\ddot{x}$','$\ddot{\hat{x}}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Acceleration');

figure; hold on; grid on;
plot(Ts, XS(4, :), 'k')
plot(Ts, mu(4, :), 'c'); 
h = legend('${z}^{(3)}$','${x}^{(3)}$','$\hat{x}^{(3)}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Jerk');


