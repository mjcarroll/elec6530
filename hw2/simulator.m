%%
clear; clc; close all;

%% Robot Parameters
right_radius = 0.5;
left_radius = 0.5;
wheel_base = 1.0;

%% Simulation Parameters

% Total simulation time is simLength * deltaT
simLength = 1000;
deltaT = 0.01;

% Origin is [time,x,y,theta]
origin = [0;0;0;0];

%wheelVels = repmat([1,1],simLength,1);
%wheelVels = repmat([1,5],simLength,1);
wheelVels = repmat([1,1],simLength,1) + randn(simLength,2);

%% Simulation

% Create an empty array of poses, and set the first pose to simulation
% origin.
pose = zeros(length(wheelVels),4);
pose(1,:) = origin;

% Calculate the pose of the robot throughout time.
for ii = 2:length(wheelVels),
    omegaL = wheelVels(ii,1);
    omegaR = wheelVels(ii,2);
    
    v = (right_radius * omegaR + left_radius * omegaL)/2;
    w = (right_radius * omegaR - left_radius * omegaL)/wheel_base;
    
    pose(ii,1) = pose(ii-1,1) + deltaT;
    pose(ii,2) = pose(ii-1,2) + (deltaT * v) * ...
        cos(pose(ii-1,4) + (deltaT * w/2));
    pose(ii,3) = pose(ii-1,3) + (deltaT * v) * ...
        sin(pose(ii-1,4) + (deltaT * w/2));
    pose(ii,4) = wrapToPi(pose(ii-1,4) + deltaT * w);
end

%% Plots
t = pose(:,1); x = pose(:,2); y = pose(:,3); theta = pose(:,4);

figure(1); 
subplot(3,1,1); plot(t,x);
xlabel('Time (s)'); ylabel('X Position (m)');
subplot(3,1,2); plot(t,y); 
xlabel('Time (s)'); ylabel('Y Position (m)');
subplot(3,1,3); plot(t,theta); 
xlabel('Time (s)'); ylabel('Orientation (rad)');
title('Pose versus Time');

figure(2); 
plot(x,y); xlabel('X Position'); ylabel('Y Position'); axis equal
title('Motion in XY Plane');