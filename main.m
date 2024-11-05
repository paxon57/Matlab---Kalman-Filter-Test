clear; clc;
%% Settings
% Simulation Settings
dt = 0.5; % seconds
max_time = 60; % seconds
map_size = 10000; % meters
% Initial Parameters
init_pos = [0, randi(map_size - 200) + 200];
ang = (rand() * 2 - 1) * pi / 3 + pi / 2;
speed = 150; % m/s
% Plane Behaviour
turn_speed = pi / 5;
turn_chance = 0.04;
acceleration = 9.81;
speed_change_chance = 0.03;
% Radar Parameters
radar_pos = [rand() * map_size, rand() * map_size];
detect_noise = 150;

%% Setup
total_samples = max_time / dt;
k = 1;
real_pos(1,:) = init_pos;
detection_pos(1, :) = real_pos(1, :) + (rand(1,2) * 2 - 1) * detect_noise;
desired_ang = ang;
desired_speed = speed;
% Kalman Filter Setup
Q = [10 0 0 0;
    0 10 0 0;
    0 0 5 0;
    0 0 0 pi/5]; % Process Covariance
R = [150 0;
    0 150]; % Measurement Covariance
estimated_state(1,:) = [map_size/2, map_size/2, 100, pi/2];
P = eye(4);

%% Main Loop
for t = dt:dt:max_time
    k = k + 1;

    % Chance to turn
    if rand < turn_chance
        desired_ang = (rand() * 2 - 1) * pi / 4 + pi / 2;
        disp(['Turning to: ', num2str(desired_ang)]);
    end
    % Turning
    ang = ang + max(min(desired_ang - ang, turn_speed * dt), -turn_speed * dt);

    % Chance to change speed
    if rand < speed_change_chance
        desired_speed = 100 + rand * 150;
        disp(['Accelerating to: ', num2str(desired_speed)]);
    end
    % Accelerating
    speed = speed + max(min(desired_speed - speed, acceleration * dt), -acceleration * dt);
    
    % Move Real Plane
    real_pos(k,:) = real_pos(k-1,:) + [dt * speed * sin(ang), dt * speed * cos(ang)];
    
    % Detection
    detection_pos(k, :) = real_pos(k, :) + (rand(1,2) * 2 - 1) * detect_noise;

    %%% Kalman Filter
    %% Prediction
    F = [1, 0, sin(estimated_state(k-1,4)) * dt, 0;
        0, 1, cos(estimated_state(k-1,4)) * dt, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
    % State Prediction
    x_pred = (F*estimated_state(k-1,:)')';
    % Update state covariance
    P_pred = F * P * F' + Q;
    %% Update
    % Observation Matrix
    H = [1, 0, 0, 0;
        0, 1, 0, 0];
    % Kalman Gain
    K = (P_pred * H') / (H*P_pred*H' + R);
    % Measurement
    Z = detection_pos(k, :)';
    % State Estimation
    x_est = x_pred' + K * (Z - H * x_pred');
    P_est = P_pred - K * (H * P_pred);

    % Update Estimation
    estimated_state(k,:) = x_est';
    P = P_est;
    
    % Draw Simulation
    drawSimulation(k, map_size, radar_pos, real_pos, detection_pos, estimated_state);
    % Kinda Real Time
    %pause(dt);
end
