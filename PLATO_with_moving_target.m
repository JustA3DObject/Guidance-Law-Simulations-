% AUV Initial Conditions
x0 = -50;     % Initial x position (m)
y0 = -50;     % Initial y position (m)
psi0 = deg2rad(225); % Initial heading (rad)
start = [x0; y0]; % Starting position vector

% Docking Target Parameters
% The target is now the center of the larger circle of the frustum.
dock_start_pos = [0; 0]; % Docking station's starting position
dock_vx = 0.5; % Dock velocity in x direction (m/s)
dock_vy = 0; % Dock velocity in y direction (m/s)
dock_velocity_vector = [dock_vx; dock_vy];

% Frustum parameters
radius1 = 1.0; % Larger radius of the frustum (m)
radius2 = 0.5; % Smaller radius of the frustum (m)
length_frustum = 3.0; % Length of the frustum (m)

% Water Current Parameters
current_v = 0.5; % Velocity of the water current (m/s)
% The current is at 90 degrees to the shortest path.
% Shortest path vector: V_path = dock_start_pos - start
V_path_shortest = dock_start_pos - start;
% Angle of the shortest path
psi_path_shortest = atan2(V_path_shortest(2), V_path_shortest(1));
% Angle of the current is 90 degrees to the shortest path
psi_current = psi_path_shortest + deg2rad(90);
% Current velocity vector components
current_vx = current_v * cos(psi_current);
current_vy = current_v * sin(psi_current);
current_vector = [current_vx; current_vy];

% AUV Motion Profile Parameters
initial_v = 0.0;      % Initial forward speed at t=0 (m/s)
initial_a = 0.8;      % Constant acceleration for the initial phase (m/s^2)
accel_duration = 5; % Duration of the initial acceleration phase (s)
% Calculate final cruise speed after acceleration phase
cruise_speed = initial_v + initial_a * accel_duration;

% AUV and Controller Parameters
% Heading PID Controller - controls the AUV's direction
Kp = 1.5;      % Proportional gain - responds to current error
Ki = 0.05;     % Integral gain - eliminates steady-state error
Kd = 0.03;     % Derivative gain - dampens oscillations

% Guidance Parameters
docking_radius = 0.1; % Radius for successful docking (m) - considered "docked" within this distance

% Parameters for Dynamic Deceleration
max_deceleration = 0.5; % Maximum comfortable deceleration rate (m/s^2)
deceleration_factor = 1.2; % Tuning factor: >1 starts deceleration earlier, <1 later
lookahead = 1.0; % Lookahead distance for Carrot Chase and NLG (m) - how far ahead to "look"
K_nlg_p = 1.0;    % Proportional gain for the robust NLG heading controller

% Deceleration PID Parameters
Kp_s = 3.0; % Proportional gain for speed control
Ki_s = 0.1; % Integral gain for speed control
Kd_s = 0.05; % Derivative gain for speed control

% Simulation Time
tspan = [0 37.3]; % Simulation time span (s)

% Initial State Vector
% State: [x; y; s; psi; integral_heading_error; previous_heading_error; integral_speed_error; previous_speed_error]
y0_full = [x0; y0; initial_v; psi0; 0; 0; 0; 0];

% Common parameters to be passed to the model
common_params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
'dock_start_pos', dock_start_pos, 'dock_velocity_vector', dock_velocity_vector, ...
'docking_radius', docking_radius, ...
'cruise_speed', cruise_speed, ...
'start', start, 'lookahead', lookahead, ...
'initial_a', initial_a, 'accel_duration', accel_duration, ...
'max_deceleration', max_deceleration, 'deceleration_factor', deceleration_factor, ...
'Kp_s', Kp_s, 'Ki_s', Ki_s, 'Kd_s', Kd_s, ...
'current_vector', current_vector, ...
'docking_speed', 0.5, ...
'frustum_length', length_frustum, 'frustum_radius1', radius1, 'frustum_radius2', radius2);

% Run simulations for each guidance algorithm
fprintf('Running LOS Guidance Simulation...\n');
params_LOS = common_params;
params_LOS.guidance = 'LOS'; % Line-of-Sight guidance
% Configure ODE solver to stop when docking event occurs
options_LOS = odeset('Events', @(t,y) docking_events(t, y, params_LOS));
[T_LOS, Y_LOS] = ode45(@(t,y) auv_model_log_poly_decel(t, y, params_LOS), tspan, y0_full, options_LOS);

fprintf('Running Carrot Chase Simulation...\n');
params_carrot = common_params;
params_carrot.guidance = 'carrot'; % Carrot Chase guidance
options_carrot = odeset('Events', @(t,y) docking_events(t, y, params_carrot));
[T_carrot, Y_carrot] = ode45(@(t,y) auv_model_log_poly_decel(t, y, params_carrot), tspan, y0_full, options_carrot);

fprintf('Running Non-Linear Guidance (NLG) Simulation...\n');
params_NLG = common_params;
params_NLG.guidance = 'NLG'; % Non-Linear Guidance
params_NLG.K_nlg_p = K_nlg_p;
options_NLG = odeset('Events', @(t,y) docking_events(t, y, params_NLG));
[T_NLG, Y_NLG] = ode45(@(t,y) auv_model_log_poly_decel(t, y, params_NLG), tspan, y0_full, options_NLG);

fprintf('Simulations complete. Plotting results...\n');

% Plot Comparison Results
plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, common_params);

% Figure 5: Relative Velocities
figure('Name', 'Relative Velocities', 'Position', [100, 100, 1200, 900]);
% Velocity components for each guidance law
vx_LOS = Y_LOS(:,3) .* cos(Y_LOS(:,4));
vy_LOS = Y_LOS(:,3) .* sin(Y_LOS(:,4));
v_mag_LOS = sqrt(vx_LOS.^2 + vy_LOS.^2);

vx_carrot = Y_carrot(:,3) .* cos(Y_carrot(:,4));
vy_carrot = Y_carrot(:,3) .* sin(Y_carrot(:,4));
v_mag_carrot = sqrt(vx_carrot.^2 + vy_carrot.^2);

vx_NLG = Y_NLG(:,3) .* cos(Y_NLG(:,4));
vy_NLG = Y_NLG(:,3) .* sin(Y_NLG(:,4));
v_mag_NLG = sqrt(vx_NLG.^2 + vy_NLG.^2);

% Relative velocity magnitude
subplot(3,1,1);
hold on;
plot(T_LOS, v_mag_LOS, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LOS');
plot(T_carrot, v_mag_carrot, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Carrot');
plot(T_NLG, v_mag_NLG, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'NLG');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Velocity Magnitude (m/s)');
title('Relative Velocity Magnitude');
legend;

% X component of relative velocity
subplot(3,1,2);
hold on;
plot(T_LOS, vx_LOS, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LOS');
plot(T_carrot, vx_carrot, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Carrot');
plot(T_NLG, vx_NLG, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'NLG');
hold off;
grid on;
xlabel('Time (s)');
ylabel('V_x (m/s)');
title('X Component of Relative Velocity');
legend;

% Y component of relative velocity
subplot(3,1,3);
hold on;
plot(T_LOS, vy_LOS, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LOS');
plot(T_carrot, vy_carrot, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Carrot');
plot(T_NLG, vy_NLG, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'NLG');
hold off;
grid on;
xlabel('Time (s)');
ylabel('V_y (m/s)');
title('Y Component of Relative Velocity');
legend;

% Figure 6: Range Rate (rdot) to Target
figure('Name', 'Range Rate (rdot)', 'Position', [100, 100, 1200, 600]);
% rdot for each guidance law using a loop
rdot_LOS = zeros(size(T_LOS));
for i = 1:length(T_LOS)
    auv_pos = [Y_LOS(i,1); Y_LOS(i,2)];
    auv_vel_body = [Y_LOS(i,3)*cos(Y_LOS(i,4)); Y_LOS(i,3)*sin(Y_LOS(i,4))];
    auv_vel_ground = auv_vel_body + common_params.current_vector;
    target_pos = get_dock_position(T_LOS(i), common_params.dock_start_pos, common_params.dock_velocity_vector);
    
    vec_to_target = target_pos - auv_pos;
    dist_to_target = norm(vec_to_target);
    if dist_to_target > 1e-6
        rdot_LOS(i) = dot(auv_vel_ground, vec_to_target) / dist_to_target;
    else
        rdot_LOS(i) = 0;
    end
end

rdot_carrot = zeros(size(T_carrot));
for i = 1:length(T_carrot)
    auv_pos = [Y_carrot(i,1); Y_carrot(i,2)];
    auv_vel_body = [Y_carrot(i,3)*cos(Y_carrot(i,4)); Y_carrot(i,3)*sin(Y_carrot(i,4))];
    auv_vel_ground = auv_vel_body + common_params.current_vector;
    target_pos = get_dock_position(T_carrot(i), common_params.dock_start_pos, common_params.dock_velocity_vector);
    
    vec_to_target = target_pos - auv_pos;
    dist_to_target = norm(vec_to_target);
    if dist_to_target > 1e-6
        rdot_carrot(i) = dot(auv_vel_ground, vec_to_target) / dist_to_target;
    else
        rdot_carrot(i) = 0;
    end
end

rdot_NLG = zeros(size(T_NLG));
for i = 1:length(T_NLG)
    auv_pos = [Y_NLG(i,1); Y_NLG(i,2)];
    auv_vel_body = [Y_NLG(i,3)*cos(Y_NLG(i,4)); Y_NLG(i,3)*sin(Y_NLG(i,4))];
    auv_vel_ground = auv_vel_body + common_params.current_vector;
    target_pos = get_dock_position(T_NLG(i), common_params.dock_start_pos, common_params.dock_velocity_vector);
    
    vec_to_target = target_pos - auv_pos;
    dist_to_target = norm(vec_to_target);
    if dist_to_target > 1e-6
        rdot_NLG(i) = dot(auv_vel_ground, vec_to_target) / dist_to_target;
    else
        rdot_NLG(i) = 0;
    end
end

% Plot the results
hold on;
plot(T_LOS, -rdot_LOS, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LOS');
plot(T_carrot, -rdot_carrot, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Carrot');
plot(T_NLG, -rdot_NLG, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'NLG');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Range Rate (m/s)');
title('Range Rate (Closing Velocity) to Target');
legend('Location', 'southeast');
yline(0, 'k--', 'DisplayName', 'Zero Closing Velocity');

% Function to get the dock's position at a given time
function pos = get_dock_position(t, dock_start_pos, dock_velocity_vector)
    pos = dock_start_pos + dock_velocity_vector * t;
end

% Event Function for Docking
function [value, isterminal, direction] = docking_events(t, y, params)
% Get the current target position
target = get_dock_position(t, params.dock_start_pos, params.dock_velocity_vector);
% Distance to target
dist_to_target = sqrt((y(1) - target(1))^2 + (y(2) - target(2))^2);
value = dist_to_target - params.docking_radius; % Event occurs when this becomes zero
isterminal = 1; % Stop the simulation when event occurs
direction = -1; % Only trigger when value is decreasing (approaching target)
end

% AUV Model
function dydt = auv_model_log_poly_decel(t, y, params)
% Unpack state vector
x = y(1); % Current x position
y_pos = y(2); % Current y position
s = y(3); % Current forward speed (relative to water)
psi = y(4); % Current heading angle
integral_heading_error = y(5); % Integral of heading error (for PID)
previous_heading_error = y(6); % Previous heading error (for derivative term)
integral_speed_error = y(7); % Integral of speed error (for PID)
previous_speed_error = y(8); % Previous speed error (for derivative term)

% Current target position (center of the larger circle)
target = get_dock_position(t, params.dock_start_pos, params.dock_velocity_vector);
current_vector = params.current_vector; % Constant current vector

% AUV's velocity vector relative to the water
v_auv_body = [s * cos(psi); s * sin(psi)];
% AUV's total velocity vector relative to the ground (water velocity + AUV's velocity)
v_auv_ground = v_auv_body + current_vector;
% Total speed relative to the ground
s_ground = norm(v_auv_ground);
% Heading relative to the ground
psi_ground = atan2(v_auv_ground(2), v_auv_ground(1));

% Heading Control
if strcmp(params.guidance, 'NLG')
    % Non-Linear Guidance
    rc = nonlinear_guidance(x, y_pos, psi, params, v_auv_ground, s_ground, psi_ground, target);
    heading_error = 0;
    d_integral_heading_error = 0;
else
    % Standard PID controller for LOS and Carrot Chase
    if strcmp(params.guidance, 'carrot')
        desired_psi_ground = carrot_guidance(x, y_pos, params, target);
    else % Default to LOS
        desired_psi_ground = atan2(target(2) - y_pos, target(1) - x);
    end

    % The guidance law calculates the desired heading relative to the ground.
    % The control system must adjust the AUV's heading (relative to water)
    % to achieve this desired ground heading.
    % First, find the desired velocity vector relative to the ground.
    v_desired_ground = s_ground * [cos(desired_psi_ground); sin(desired_psi_ground)];
    
    % Then, find the required AUV body velocity vector by subtracting the current.
    v_auv_body_desired = v_desired_ground - current_vector;
    
    % The desired heading is the angle of this body velocity vector.
    desired_psi = atan2(v_auv_body_desired(2), v_auv_body_desired(1));

    % Heading error (wrapped to [-π, π])
    heading_error = wrapToPi(desired_psi - psi);
    d_integral_heading_error = heading_error;

    % Derivative of heading error
    derivative_heading_error = heading_error - previous_heading_error;

    % PID control law for heading
    rc = params.Kp * heading_error + params.Ki * integral_heading_error + params.Kd * derivative_heading_error;
end

% Limit the maximum angle for correcting the heading angle to be between 30 and -30 degrees.
% This is applied to the output of the PID/guidance controller.
max_correction_rad = deg2rad(30);
rc = max(-max_correction_rad, min(max_correction_rad, rc));

% Speed Control - Log Polynomial Law for Deceleration
dist_to_target = sqrt((x - target(1))^2 + (y_pos - target(2))^2);

% Dynamic Deceleration Logic
% Dynamic deceleration radius based on current ground speed, max
% deceleration and docking radius
deceleration_distance = params.deceleration_factor * (s_ground^2 / (2 * params.max_deceleration));
dynamic_decel_radius = params.docking_radius + deceleration_distance;

% Initialize speed PID variables for all cases
d_integral_speed_error = 0;
speed_error = 0;

if t < params.accel_duration
    % Phase 1: Initial constant acceleration
    % This acceleration is applied to the AUV's speed relative to water
    ds = params.initial_a;
elseif dist_to_target > dynamic_decel_radius
    % Phase 2: Cruising at constant speed
    ds = 0;
else
    % Phase 3: Deceleration using Dynamic Log Polynomial Law
    % Calculate the desired ground speed
    if dist_to_target <= params.docking_radius
        % When docking, the desired ground velocity is the current velocity plus a small docking velocity
        % in the direction of the target.
        if dist_to_target > 0
            v_docking_des = params.docking_speed * (target - [x; y_pos]) / dist_to_target;
        else
            v_docking_des = [0;0];
        end
        s_desired = norm(current_vector + v_docking_des);
    else
        % Dynamically determine the exponent 'n' based on speed.
        % We use the ground speed for this calculation to account for the current.
        n_min = 2;
        n_max = 5;
        % Normalize current ground speed relative to cruise speed
        speed_ratio = min(1, s_ground / params.cruise_speed);
        n = n_min + (n_max - n_min) * speed_ratio;

        term = (log(dist_to_target)) / ...
               (log(dynamic_decel_radius));
        
        % The desired speed is relative to the ground
        s_desired_ground = params.cruise_speed * max(0, term)^n;
        
        % The desired speed relative to the water must be calculated
        % to achieve this desired ground speed.
        v_auv_body_desired = s_desired_ground * [cos(psi_ground); sin(psi_ground)] - current_vector;
        s_desired = norm(v_auv_body_desired);
    end

    % PID control for speed relative to water
    speed_error = s_desired - s;
    d_integral_speed_error = speed_error;
    derivative_speed_error = speed_error - previous_speed_error;

    % PID control law for acceleration/deceleration
    ds = params.Kp_s * speed_error + params.Ki_s * integral_speed_error + params.Kd_s * derivative_speed_error;
    % Clamp the acceleration/deceleration to a max value to prevent overshoots
    ds = max(-params.max_deceleration, min(params.initial_a, ds));
end

% Equations of Motion (all velocities are relative to the ground)
dx = s * cos(psi) + current_vector(1); % x velocity component
dy = s * sin(psi) + current_vector(2); % y velocity component
dpsi = rc; % Heading rate of change

% Assemble derivative vector for ODE solver
dydt = [dx; dy; ds; dpsi; d_integral_heading_error; heading_error; d_integral_speed_error; speed_error];
end

% Non-Linear Guidance (Robust Lookahead Version)
function rc = nonlinear_guidance(x, y_pos, psi, params, v_auv_ground, s_ground, psi_ground, target)
    start = params.start;
    delta = params.lookahead; % Lookahead distance
    Kp = params.K_nlg_p; % Proportional gain
    auv_pos = [x; y_pos];
    
    % The path is from the AUV's start position to the target
    path_vec = target - start;
    
    carrot_point = target; % Default carrot to the end of the path
    
    if dot(path_vec, path_vec) > 1e-10
        % Projection of current position onto path
        projection_ratio = dot(auv_pos - start, path_vec) / dot(path_vec, path_vec);
        projection_point = start + max(0, min(1, projection_ratio)) * path_vec;
        
        % Lookahead point
        dist_remaining = norm(target - projection_point);
        lookahead_dist = min(delta, dist_remaining);
        if dist_remaining > 1e-6
            carrot_point = projection_point + lookahead_dist * (path_vec / norm(path_vec));
        end
    end

    % The desired course is to the lookahead point. This is the desired ground-relative heading.
    desired_course_ground = atan2(carrot_point(2) - y_pos, carrot_point(1) - x);

    % The heading error is calculated between the desired ground heading and the
    % current ground heading.
    course_error = wrapToPi(desired_course_ground - psi_ground);

    % The control law determines the rate of change of the AUV's body heading to
    % correct for this error. This proportional law is an approximation.
    rc = Kp * course_error;
end

% Carrot Guidance
function desired_psi = carrot_guidance(x, y_pos, params, target)
    start = params.start;
    L = params.lookahead; % Lookahead distance
    P = [x; y_pos]; % Current position
    A = start;
    B = target;
    
    V = B - A; % Path vector
    if dot(V, V) < 1e-10
        carrot = B; % If path is degenerate, aim directly at target
    else
        % Find closest point on path to current position
        Q = A + max(0, min(1, dot(P - A, V) / dot(V, V))) * V;
        
        % Vector from Q to target
        to_target = B - Q;
        d = norm(to_target);
        if d > 0
            % Place carrot at lookahead distance along path
            carrot = Q + min(L, d) * (to_target / d);
        else
            carrot = Q; % If at target, stay there
        end
    end
    
    % The desired heading is to the carrot point. This is the desired ground-relative heading.
    desired_psi = atan2(carrot(2) - y_pos, carrot(1) - x);
end

% Plotting and Helper Functions
function plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, params)
% Extract parameters
docking_radius = params.docking_radius;
lw = 1.5; % Line width for plots
current_vector = params.current_vector;
start = params.start;
dock_start_pos = params.dock_start_pos;
dock_velocity_vector = params.dock_velocity_vector;

nlg_color = [0.8500 0.3250 0.0980]; % Orange

% Extract data
x_LOS = Y_LOS(:,1); y_LOS = Y_LOS(:,2); s_LOS = Y_LOS(:,3); psi_LOS = Y_LOS(:,4);
x_carrot = Y_carrot(:,1); y_carrot = Y_carrot(:,2); s_carrot = Y_carrot(:,3); psi_carrot = Y_carrot(:,4);
x_NLG = Y_NLG(:,1); y_NLG = Y_NLG(:,2); s_NLG = Y_NLG(:,3); psi_NLG = Y_NLG(:,4);

% Ground-relative velocities for plotting
vx_LOS_ground = s_LOS .* cos(psi_LOS) + current_vector(1);
vy_LOS_ground = s_LOS .* sin(psi_LOS) + current_vector(2);
vx_carrot_ground = s_carrot .* cos(psi_carrot) + current_vector(1);
vy_carrot_ground = s_carrot .* sin(psi_carrot) + current_vector(2);
vx_NLG_ground = s_NLG .* cos(psi_NLG) + current_vector(1);
vy_NLG_ground = s_NLG .* sin(psi_NLG) + current_vector(2);

% Dynamically calculated deceleration radius at cruise speed for plotting
deceleration_distance_at_cruise = params.deceleration_factor * (params.cruise_speed^2 / (2 * params.max_deceleration));
dynamic_decel_radius_at_cruise = params.docking_radius + deceleration_distance_at_cruise;

% Figure 1: Trajectory Comparison
figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 800, 600]);
hold on;
% Plot trajectories for each guidance method
plot(x_LOS, y_LOS, 'r-', 'LineWidth', lw, 'DisplayName', 'LOS');
plot(x_carrot, y_carrot, 'g-', 'LineWidth', lw, 'DisplayName', 'Carrot');
plot(x_NLG, y_NLG, 'Color', nlg_color, 'LineWidth', lw, 'DisplayName', 'NLG');

% Plot reference elements
plot(start(1), start(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'AUV Start');

% Plot the moving dock's path
T_dock = 0:0.1:max([T_LOS(end), T_carrot(end), T_NLG(end)]);
dock_path_x = dock_start_pos(1) + dock_velocity_vector(1) * T_dock;
dock_path_y = dock_start_pos(2) + dock_velocity_vector(2) * T_dock;
plot(dock_path_x, dock_path_y, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Dock Path');

% Plot the final docking position
final_target_LOS = get_dock_position(T_LOS(end), dock_start_pos, dock_velocity_vector);
final_target_carrot = get_dock_position(T_carrot(end), dock_start_pos, dock_velocity_vector);
final_target_NLG = get_dock_position(T_NLG(end), dock_start_pos, dock_velocity_vector);

% plot(final_target_LOS(1), final_target_LOS(2), 'r*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'LOS End');
% plot(final_target_carrot(1), final_target_carrot(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Carrot End');
% plot(final_target_NLG(1), final_target_NLG(2), 'Color', nlg_color, 'Marker', '*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'NLG End');

% Plot the frustum at the final position of Carrot Chase simulation.
final_time_carrot = T_carrot(end);
target_pos_at_end = get_dock_position(final_time_carrot, dock_start_pos, dock_velocity_vector);

% Direction of motion of the dock (and the frustum)
motion_unit_vec = dock_velocity_vector / norm(dock_velocity_vector);

% The larger circle is aligned with the target and the smaller is in the direction of motion
center_large_circle = target_pos_at_end;
center_small_circle = target_pos_at_end + params.frustum_length * motion_unit_vec;

% Points for the frustum's side view
normal_unit = [-motion_unit_vec(2); motion_unit_vec(1)]; % Normal vector to the motion path
p1 = center_large_circle + params.frustum_radius1 * normal_unit;
p2 = center_large_circle - params.frustum_radius1 * normal_unit;
p3 = center_small_circle - params.frustum_radius2 * normal_unit;
p4 = center_small_circle + params.frustum_radius2 * normal_unit;

% Plot the frustum as a patch
x_frustum = [p1(1), p2(1), p3(1), p4(1), p1(1)];
y_frustum = [p1(2), p2(2), p3(2), p4(2), p1(2)];
fill(x_frustum, y_frustum, [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Docking Frustum');

% Plot the shortest path line passing through the centers
plot([center_large_circle(1), center_small_circle(1)], [center_large_circle(2), center_small_circle(2)], 'k-.', 'LineWidth', 1.0, 'DisplayName', 'Frustum Centerline');

% Dock's starting position
plot(dock_start_pos(1), dock_start_pos(2), 'kd', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', 'Dock Start');

% Dock's ending position
plot(target_pos_at_end(1), target_pos_at_end(2), 'k^', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'c', 'DisplayName', 'Dock End');

hold off;
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('AUV Trajectory Comparison with Moving Dock');
legend('Location', 'best');

% Figure 2: Performance Metrics Comparison
figure('Name', 'Performance Metrics Comparison', 'Position', [950, 100, 800, 700]);
% Distance to target for each method
dist_error_LOS = arrayfun(@(t,x,y) norm(get_dock_position(t, dock_start_pos, dock_velocity_vector) - [x;y]), T_LOS, x_LOS, y_LOS);
dist_error_carrot = arrayfun(@(t,x,y) norm(get_dock_position(t, dock_start_pos, dock_velocity_vector) - [x;y]), T_carrot, x_carrot, y_carrot);
dist_error_NLG = arrayfun(@(t,x,y) norm(get_dock_position(t, dock_start_pos, dock_velocity_vector) - [x;y]), T_NLG, x_NLG, y_NLG);

% Distance to target over time
subplot(2,2,1);
hold on;
plot(T_LOS, dist_error_LOS, 'r-', 'LineWidth', lw);
plot(T_carrot, dist_error_carrot, 'g-', 'LineWidth', lw);
plot(T_NLG, dist_error_NLG, 'Color', nlg_color, 'LineWidth', lw);
% Add reference lines for deceleration radii
yline(dynamic_decel_radius_at_cruise, 'y-.', 'Decel. Zone (Approx)');
hold off; grid on;
xlabel('Time (s)'); ylabel('Distance (m)');
title('Distance to Target');
legend('LOS', 'Carrot', 'NLG', 'Location', 'northeast');

% Heading error comparison (Ground Heading Error)
subplot(2,2,2);
hold on;
% Ground heading for each method
psi_LOS_ground = atan2(vy_LOS_ground, vx_LOS_ground);
psi_carrot_ground = atan2(vy_carrot_ground, vx_carrot_ground);
psi_NLG_ground = atan2(vy_NLG_ground, vx_NLG_ground);

% Desired ground heading for each method
desired_psi_LOS = zeros(size(T_LOS));
for i = 1:length(T_LOS)
    target = get_dock_position(T_LOS(i), dock_start_pos, dock_velocity_vector);
    desired_psi_LOS(i) = atan2(target(2) - y_LOS(i), target(1) - x_LOS(i));
end
desired_psi_carrot = zeros(size(T_carrot));
for i = 1:length(T_carrot)
    target = get_dock_position(T_carrot(i), dock_start_pos, dock_velocity_vector);
    desired_psi_carrot(i) = carrot_guidance(x_carrot(i), y_carrot(i), params, target);
end
desired_psi_NLG = zeros(size(T_NLG));
for i = 1:length(T_NLG)
    target = get_dock_position(T_NLG(i), dock_start_pos, dock_velocity_vector);
    desired_psi_NLG(i) = atan2(target(2) - y_NLG(i), target(1) - x_NLG(i));
end

plot(T_LOS, rad2deg(wrapToPi(desired_psi_LOS - psi_LOS_ground)), 'r-', 'LineWidth', lw);
plot(T_carrot, rad2deg(wrapToPi(desired_psi_carrot - psi_carrot_ground)), 'g-', 'LineWidth', lw);
plot(T_NLG, rad2deg(wrapToPi(desired_psi_NLG - psi_NLG_ground)), 'Color', nlg_color, 'LineWidth', lw);
hold off; grid on;
xlabel('Time (s)'); ylabel('Ground Heading Error (deg)');
title('Ground Heading Error Comparison');
ylim([-190, 190]); % Limit y-axis to reasonable range

% Path following performance (cross-track error)
subplot(2,2,3);
% Calculate cross-track error using a loop for robustness
cross_track_LOS = zeros(size(T_LOS));
for i = 1:length(T_LOS)
    target = get_dock_position(T_LOS(i), dock_start_pos, dock_velocity_vector);
    cross_track_LOS(i) = calculate_cross_track(x_LOS(i), y_LOS(i), start, target);
end
cross_track_carrot = zeros(size(T_carrot));
for i = 1:length(T_carrot)
    target = get_dock_position(T_carrot(i), dock_start_pos, dock_velocity_vector);
    cross_track_carrot(i) = calculate_cross_track(x_carrot(i), y_carrot(i), start, target);
end
cross_track_NLG = zeros(size(T_NLG));
for i = 1:length(T_NLG)
    target = get_dock_position(T_NLG(i), dock_start_pos, dock_velocity_vector);
    cross_track_NLG(i) = calculate_cross_track(x_NLG(i), y_NLG(i), start, target);
end
hold on;
plot(T_LOS, cross_track_LOS, 'r-', 'LineWidth', lw);
plot(T_carrot, cross_track_carrot, 'g-', 'LineWidth', lw);
plot(T_NLG, cross_track_NLG, 'Color', nlg_color, 'LineWidth', lw);
hold off; grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('Path Following Performance');

% Control effort (turn rate)
subplot(2,2,4);
hold on;
% Turn rate for each method (derivative of heading relative to water)
plot(T_LOS, rad2deg([0; diff(Y_LOS(:,4)) ./ diff(T_LOS)]), 'r-', 'LineWidth', lw);
plot(T_carrot, rad2deg([0; diff(Y_carrot(:,4)) ./ diff(T_carrot)]), 'g-', 'LineWidth', lw);
plot(T_NLG, rad2deg([0; diff(Y_NLG(:,4)) ./ diff(T_NLG)]), 'Color', nlg_color, 'LineWidth', lw);
hold off; grid on;
xlabel('Time (s)'); ylabel('Turn Rate (deg/s)');
title('Control Effort Comparison');

% Add overall title to the figure
sgtitle('AUV Guidance: Core Performance Metrics with Current', 'FontSize', 14, 'FontWeight', 'bold');

% Figure 3: Speed and Efficiency Metrics
figure('Name', 'Speed and Efficiency Metrics', 'Position', [100, 50, 900, 700]);
% Create categorical array for guidance methods
labels = categorical({'LOS', 'Carrot', 'NLG'});
labels = reordercats(labels, {'LOS', 'Carrot', 'NLG'});

% Extract simulation end times and path lengths
times = [T_LOS(end), T_carrot(end), T_NLG(end)];
path_lengths = [calculate_path_length(x_LOS, y_LOS), ...
                calculate_path_length(x_carrot, y_carrot), ...
                calculate_path_length(x_NLG, y_NLG)];

% Speed profiles
subplot(2,2,[1,2]);
hold on;
plot(T_LOS, s_LOS, 'r-', 'LineWidth', lw, 'DisplayName', 'LOS');
plot(T_carrot, s_carrot, 'g-', 'LineWidth', lw, 'DisplayName', 'Carrot');
plot(T_NLG, s_NLG, 'Color', nlg_color, 'LineWidth', lw, 'DisplayName', 'NLG');
grid on; hold off;
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('AUV Speed Profile (Relative to Water)');
legend('LOS', 'Carrot', 'NLG', 'Location', 'best');

% Time to target bar chart
subplot(2,2,3);
bar(labels, times);
grid on;
title('Time to Reach Target');
ylabel('Time (s)');

% Path length bar chart
subplot(2,2,4);
bar(labels, path_lengths);
grid on;
title('Total Path Length');
ylabel('Distance (m)');
% Reference line for minimum possible distance
yline(norm(final_target_LOS - start), 'r--', 'Min. Distance (LOS)');

% Overall title to the figure
sgtitle('AUV Guidance: Speed and Efficiency with Current', 'FontSize', 14, 'FontWeight', 'bold');

% Figure 4: Deceleration Performance Analysis
figure('Name', 'Deceleration Performance Analysis', 'Position', [950, 50, 900, 400]);
% Final speeds and distances
s_final_LOS_ground = norm([vx_LOS_ground(end), vy_LOS_ground(end)]);
s_final_carrot_ground = norm([vx_carrot_ground(end), vy_carrot_ground(end)]);
s_final_NLG_ground = norm([vx_NLG_ground(end), vy_NLG_ground(end)]);

final_speeds = [s_final_LOS_ground, s_final_carrot_ground, s_final_NLG_ground];
final_distances = [dist_error_LOS(end), dist_error_carrot(end), dist_error_NLG(end)];

% Find when deceleration starts for each method
all_dist_errors = {dist_error_LOS, dist_error_carrot, dist_error_NLG};
all_times = {T_LOS, T_carrot, T_NLG};
decel_start_times = NaN(1, 3); % Initialize with NaN
for i = 1:3
    idx = find(all_dist_errors{i} <= dynamic_decel_radius_at_cruise, 1, 'first');
    if ~isempty(idx)
        decel_start_times(i) = all_times{i}(idx);
    end
end

% Final approach performance (speed vs distance)
subplot(1, 2, 1);
markers = {'o', 's', 'p'}; % Different markers for each method
colors = {'r', 'g', nlg_color}; % Colors for each method
labels = categorical({'LOS', 'Carrot', 'NLG'});
labels = reordercats(labels, {'LOS', 'Carrot', 'NLG'});
hold on;
for i = 1:length(labels)
    plot(final_distances(i), final_speeds(i), ...
         'Marker', markers{i}, ...
         'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', colors{i}, ...
         'MarkerSize', 10, ...
         'DisplayName', char(labels(i)));
end
hold off;
grid on;
title('Final Ground Speed vs. Final Distance');
xlabel('Final Distance to Target (m)');
ylabel('Final Ground Speed (m/s)');
legend('show', 'Location', 'northeast');
% Reference line for docking radius
xline(docking_radius, 'k--', 'Target Radius');
yline(norm(current_vector) + params.docking_speed, 'r--', 'Desired Final Speed');

% Deceleration start time bar chart
subplot(1, 2, 2);
b_decel = bar(labels, decel_start_times);
grid on;
title('Deceleration Start Time');
ylabel('Time (s)');
% Text labels only for valid (non-NaN) data points
valid_indices = ~isnan(decel_start_times);
if any(valid_indices)
    text(b_decel.XEndPoints(valid_indices), b_decel.YEndPoints(valid_indices), ...
         string(round(b_decel.YData(valid_indices), 1)), ...
         'HorizontalAlignment','center', 'VerticalAlignment','bottom', 'Color', 'k');
end

% Overall title to the figure
sgtitle('AUV Guidance: Deceleration Analysis with Current', 'FontSize', 14, 'FontWeight', 'bold');
end

% Helper function: Wrap angle to [-π, π] range
function wrapped = wrapToPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end

% Helper function: Calculate path length from coordinates
function len = calculate_path_length(x_coords, y_coords)
    if numel(x_coords) < 2
        len = 0;
        return;
    end
    % Sum of distances between consecutive points
    len = sum(sqrt(diff(x_coords).^2 + diff(y_coords).^2));
end

% Helper function: Calculate cross-track error
function cross_track = calculate_cross_track(x_pos, y_pos, start, target)
    path_vec = target - start; % Vector along desired path
    path_length = norm(path_vec);
    if path_length < 1e-10
        % If path is degenerate, use distance to start point
        cross_track = norm([x_pos; y_pos] - start);
        return;
    end
    path_unit = path_vec / path_length; % Unit vector along path
    normal_vec = [-path_unit(2); path_unit(1)]; % Normal vector to path
    % Vector from start to current position
    pos_vec = [x_pos; y_pos] - start;
    % Cross-track error is the absolute dot product with normal vector
    cross_track = abs(dot(pos_vec, normal_vec));
end
