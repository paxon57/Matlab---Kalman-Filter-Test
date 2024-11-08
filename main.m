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
turn_chance = 0.05;
acceleration = 9.81;
speed_change_chance = 0.03; %0.03
% Radar Parameters
radar_pos = [rand() * map_size, rand() * map_size];
range_noise = 20;
angle_noise = deg2rad(0.5);

%% Setup
total_samples = max_time / dt;
k = 1;
real_pos(1,:) = init_pos;
detection_pos(1, :) = real_pos(1, :) + (rand(1,2) * 2 - 1) * range_noise;
desired_ang = ang;
desired_speed = speed;
error = zeros(total_samples, 3);
Z_hist = zeros(total_samples, 2);
% Kalman Filter Setup
Q = [10 0 0 0;
    0 10 0 0;
    0 0 8.0228 0;
    0 0 0 0.00823]; % Process Covariance
%Q = eye(4) * 0.5;
R = [121.4797 -0.00665;
    -0.00665 0.0000288]; % Measurement Covariance
P = eye(4);
estimated_state(1,:) = [real_pos(1, :) + rand(1,2) * 20, 150, pi];

%% Main Loop
for t = dt:dt:max_time
    k = k + 1;

    % Chance to turn
    if rand < turn_chance
        desired_ang = (rand() * 2 - 1) * pi / 4 + pi / 2;
        disp(['t = ', num2str(t), 's - Turning to: ', num2str(desired_ang)]);
    end
    % Turning
    ang = ang + max(min(desired_ang - ang, turn_speed * dt), -turn_speed * dt);

    % Chance to change speed
    if rand < speed_change_chance
        desired_speed = 100 + rand * 150;
        disp(['t = ', num2str(t), 's - Accelerating to: ', num2str(desired_speed)]);
    end
    % Accelerating
    speed = speed + max(min(desired_speed - speed, acceleration * dt), -acceleration * dt);
    
    % Move Real Plane
    real_pos(k,:) = real_pos(k-1,:) + [dt * speed * sin(ang), dt * speed * cos(ang)];
    
    %%% Kalman Filter
    %% Prediction
    estim = estimated_state(k-1,:);
    F = [1, 0, sin(estim(4)) * dt, dt * estim(3) * cos(estim(4));
        0, 1, cos(estim(4)) * dt, -dt * estim(3) * sin(estim(4));
        0, 0, 1, 0;
        0, 0, 0, 1];
    % State Prediction
    x_pred = prediction(estim, dt)';
    % Update state covariance
    P_pred = F * P * F' + Q;
    %% Update
    % Observation Matrix (Jacobian)
    diff = x_pred(1:2)' - radar_pos';
    H = [diff(1) / sqrt(diff(1)^2 + diff(2)^2), diff(2) / sqrt(diff(2)^2 + diff(1)^2), 0, 0;
        diff(2) / (diff(1)^2 + diff(2)^2), -(diff(1) / (diff(2)^2 + diff(1)^2)), 0, 0];
    % Kalman Gain
    K = (P_pred * H') / (H*P_pred*H' + R);
    % Simulated Measurements
    Z = measurement(real_pos(k, :)', radar_pos', [range_noise, angle_noise]);
    y = Z - measurement(x_pred(1:2)', radar_pos', [0 0]);
    % Save history of real measurements
    Z_hist(k, :) = Z';
    % State Estimation
    x_est = x_pred' + K * y;
    P_est = (eye(4) - K*H)*P_pred;

    % Update Estimation
    estimated_state(k,:) = x_est';
    P = P_est;
    
    % Save Error
    err_dist = real_pos(k, :) - estimated_state(k, 1:2);
    error(k, 1) = sqrt(err_dist(1)^2 + err_dist(2)^2);
    error(k, 2) = speed - estimated_state(k, 3);
    error(k, 3) = ang - estimated_state(k, 4);

    % Detection Visualization
    detection_pos(k, :) = [radar_pos(1) + Z(1) * sin(Z(2));
        radar_pos(2) + Z(1) * cos(Z(2))];

    % Draw Simulation
    drawSimulation(k, map_size, radar_pos, real_pos, detection_pos, estimated_state);
    % Kinda Real Time
    %pause(dt);
end
% Fix first sample missing
Z_hist(1, :) = Z_hist(2, :);

%% Plot Errors Over Time
t = 10:dt:max_time;
k_10s = 10 / dt + 1;
figure(2);
clf;
% Distance Error
subplot(2,2,[1 2]);
plot(t, error(k_10s:end, 1)');
xlabel("Time (s)");
ylabel("Distance (m)");
title("Position Error");
grid on;
% Velocity Error
subplot(2,2,3);
plot(t, error(k_10s:end, 2)');
xlabel("Time (s)");
ylabel("Velocity (m/s)");
title("Velocity Error");
grid on;
% Heading Error
% Temp fix to bring error to 0 from multiplies of 360 %%%%%%%%
mul = round(mean(rad2deg(error(k_10s:end, 3)')) / 360);
error(:, 3) = error(:, 3) - mul * 2 * pi;

subplot(2,2,4);
plot(t, rad2deg(error(k_10s:end, 3)'));
xlabel("Time (s)");
ylabel("Heading (deg)");
title("Heading Error");
grid on;