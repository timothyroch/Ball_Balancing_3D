%% 3D Ball Balancing on an Inverted Hemispherical Bowl using Dual PID Control
% This simulation models a ball balancing on an inverted hemispherical bowl.
% Two independent PID controllers compute the bowl's tilt angles (phi_x and phi_y)
% to keep the ball near the top (x = 0, y = 0) of the bowl.
%
% The ball dynamics (in the bowl's local frame) are modeled by:
%   x'' = (g/R)*x - g*phi_x - damping*x'
%   y'' = (g/R)*y - g*phi_y - damping*y'
%
% The bowl (local coordinates) is given by:
%   z = sqrt(R^2 - x^2 - y^2) - R, for x^2+y^2 <= R^2.
%
% At each simulation step, the bowl is rotated in the world frame by:
%   R_world = R_y(phi_x)*R_x(phi_y),
% and the ball's position is updated accordingly.

clear; clc; close all;

%% Simulation Parameters
dt      = 0.01;         % time step (s)
t_final = 20;           % total simulation time (s)
time    = 0:dt:t_final; % time vector

%% System Parameters
R       = 0.2;          % radius of hemispherical bowl (m)
g       = 9.81;         % gravitational acceleration (m/s^2)
damping = 0.1;          % damping coefficient

%% PID Controller Parameters (same for x and y)
Kp = 15;    % proportional gain
Ki = 1;     % integral gain
Kd = 5;     % derivative gain

%% Initial Conditions (ball state in the bowl’s local coordinates)
x      = 0.05;   % initial displacement in x (m)
y      = -0.05;  % initial displacement in y (m)
x_dot  = 0;      % initial velocity in x (m/s)
y_dot  = 0;      % initial velocity in y (m/s)

% PID state variables for x and y
err_int_x = 0;   prev_err_x = x;
err_int_y = 0;   prev_err_y = y;

%% Precompute the Bowl Surface (in local coordinates)
nTheta = 40; 
nPsi   = 50;
theta = linspace(0,pi/2,nTheta);
psi   = linspace(0,2*pi,nPsi);
[THETA, PSI] = meshgrid(theta, psi);
X_bowl_local = R * sin(THETA) .* cos(PSI);
Y_bowl_local = R * sin(THETA) .* sin(PSI);
Z_bowl_local = R * cos(THETA) - R;  

%% Precompute a Sphere for the Ball
ball_radius = 0.02;  % ball radius (m)
[nBall, mBall] = deal(20);
[X_ball_unit, Y_ball_unit, Z_ball_unit] = sphere(nBall);
X_ball_unit = ball_radius * X_ball_unit;
Y_ball_unit = ball_radius * Y_ball_unit;
Z_ball_unit = ball_radius * Z_ball_unit;

%% Set Up the Figure and Axes
figure('Name','3D Ball Balancing on an Inverted Bowl','NumberTitle','off');

% Top subplot: 3D Animation
subplot(2,1,1);
hold on; axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Animation of Ball on Tilted Bowl');
view(45,30);
axis_lim = R + 0.1;
xlim([-axis_lim, axis_lim]); ylim([-axis_lim, axis_lim]);
zlim([-R-0.05, 0.1]);

% Initial bowl surface (world coordinates) and ball surface.
bowl_surf = surf(X_bowl_local, Y_bowl_local, Z_bowl_local, ...
    'FaceAlpha', 0.5, 'EdgeColor','none', 'FaceColor',[0.8 0.8 1]);
ball_surf = surf(X_ball_unit, Y_ball_unit, Z_ball_unit, ...
    'FaceColor','r', 'EdgeColor','none');

% Bottom subplot: Tilt Angles over Time
subplot(2,1,2);
hold on; grid on;
phi_x_plot = plot(0,0,'b','LineWidth',2);
phi_y_plot = plot(0,0,'r','LineWidth',2);
xlabel('Time (s)'); ylabel('Tilt Angle (rad)');
title('Bowl Tilt Angles Over Time');
legend('\phi_x (about y-axis)', '\phi_y (about x-axis)');
xlim([0 t_final]); ylim([-0.2 0.2]);

%% Simulation Loop
phi_x_history = zeros(size(time));
phi_y_history = zeros(size(time));
for i = 1:length(time)
    %% PID Control for x and y
    err_x = x;   err_y = y;
    err_int_x = err_int_x + err_x * dt;
    err_int_y = err_int_y + err_y * dt;
    err_dot_x = (err_x - prev_err_x) / dt;
    err_dot_y = (err_y - prev_err_y) / dt;
    phi_x = Kp * err_x + Ki * err_int_x + Kd * err_dot_x;
    phi_y = Kp * err_y + Ki * err_int_y + Kd * err_dot_y;
    
    %% Update Ball Dynamics (local frame)
    x_ddot = (g/R)*x - g*phi_x - damping*x_dot;
    y_ddot = (g/R)*y - g*phi_y - damping*y_dot;
    x_dot = x_dot + x_ddot*dt;
    y_dot = y_dot + y_ddot*dt;
    x = x + x_dot*dt;
    y = y + y_dot*dt;
    
    %% Store PID outputs for plotting
    phi_x_history(i) = phi_x;
    phi_y_history(i) = phi_y;
    
    %% Compute the Rotation Matrix to Tilt the Bowl
    % Rotation about y-axis by phi_x and about x-axis by phi_y:
    R_x = [1,        0,           0;
           0, cos(phi_y), -sin(phi_y);
           0, sin(phi_y),  cos(phi_y)];
    R_y = [ cos(phi_x), 0, sin(phi_x);
                  0,    1,       0;
           -sin(phi_x), 0, cos(phi_x)];
    R_world = R_y * R_x;
    
    %% Transform Bowl Surface to World Coordinates
    pts_local = [X_bowl_local(:)'; Y_bowl_local(:)'; Z_bowl_local(:)'];
    pts_world = R_world * pts_local;
    X_bowl_world = reshape(pts_world(1,:), size(X_bowl_local));
    Y_bowl_world = reshape(pts_world(2,:), size(Y_bowl_local));
    Z_bowl_world = reshape(pts_world(3,:), size(Z_bowl_local));
    
    %% Compute Ball's Position in Local Coordinates
    z_contact = sqrt(max(R^2 - x^2 - y^2, 0)) - R;
    ball_contact_local = [x; y; z_contact];
    n_local = (ball_contact_local - [0;0;-R]) / R;
    ball_center_local = ball_contact_local + ball_radius * n_local;
    
    %% Transform Ball Center to World Coordinates
    ball_center_world = R_world * ball_center_local;
    
    %% Update Ball Sphere for Animation
    X_ball_world = X_ball_unit + ball_center_world(1);
    Y_ball_world = Y_ball_unit + ball_center_world(2);
    Z_ball_world = Z_ball_unit + ball_center_world(3);
    
    %% Update Graphics
    subplot(2,1,1);
    set(bowl_surf, 'XData', X_bowl_world, 'YData', Y_bowl_world, 'ZData', Z_bowl_world);
    set(ball_surf, 'XData', X_ball_world, 'YData', Y_ball_world, 'ZData', Z_ball_world);
    subplot(2,1,2);
    set(phi_x_plot, 'XData', time(1:i), 'YData', phi_x_history(1:i));
    set(phi_y_plot, 'XData', time(1:i), 'YData', phi_y_history(1:i));
    
    drawnow; pause(0.01);
    
    %% Prepare for Next Iteration
    prev_err_x = err_x;
    prev_err_y = err_y;
end
