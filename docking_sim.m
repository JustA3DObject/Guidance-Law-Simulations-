% AUV Initial Conditions
x0 = -100;     % Initial x position (m)
y0 = -100;     % Initial y position (m)
psi0 = deg2rad(225); % Initial heading (rad)
start = [x0; y0]; % Starting position vector

% Docking Target Point
xt = 0;        % Target x position (m)
yt = 0;        % Target y position (m)
target = [xt; yt]; % Target position vector

% AUV Motion Profile Parameters
initial_v = 1.0;      % Initial forward speed at t=0 (m/s)
initial_a = 2;      % Constant acceleration for the initial phase (m/s^2)
accel_duration = 5; % Duration of the initial acceleration phase (s)
% Calculate final cruise speed after acceleration phase
cruise_speed = initial_v + initial_a * accel_duration;

% AUV and Controller Parameters

% Heading PID Controller - controls the AUV's direction
Kp = 1.5;      % Proportional gain - responds to current error
Ki = 0.05;     % Integral gain - eliminates steady-state error
Kd = 0.03;     % Derivative gain - dampens oscillations

% Speed PID Controller (for deceleration) - controls AUV's speed
Kp_s = 3.0;    % Proportional gain for speed
Ki_s = 0.1;    % Integral gain for speed
Kd_s = 0.05;   % Derivative gain for speed

% Guidance Parameters
N_gain = 4.0;  % Proportional Navigation constant for TPN/RTPN guidance laws
docking_radius = 5.0; % Radius for successful docking (m) - considered "docked" within this distance
deceleration_radius = 40.0; % Radius at which to start decelerating (m)
lookahead = 1.0; % Lookahead distance for Carrot Chase and NLG (m) - how far ahead to "look"
K_nlg_p = 1.0;    % Proportional gain for the robust NLG heading controller

% Simulation Time
tspan = [0 50]; % Simulation time span (s)

% Initial State Vector
% State: [x; y; s; psi; integral_heading_error; previous_heading_error; integral_speed_error; previous_speed_error]
y0_full = [x0; y0; initial_v; psi0; 0; 0; 0; 0];

% Simulation Setup
% Configure ODE solver to stop when docking event occurs
options = odeset('Events', @(t,y) docking_events(t, y, target, docking_radius));

% Common parameters to be passed to the model
common_params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
                       'Kp_s', Kp_s, 'Ki_s', Ki_s, 'Kd_s', Kd_s, ...
                       'target', target, 'docking_radius', docking_radius, ...
                       'deceleration_radius', deceleration_radius, ...
                       'cruise_speed', cruise_speed, ...
                       'start', start, 'lookahead', lookahead, ...
                       'initial_a', initial_a, 'accel_duration', accel_duration, ...
                       'N_gain', N_gain);

% Run simulations for each guidance algorithm
fprintf('Running LOS Guidance Simulation...\n');
params_LOS = common_params;
params_LOS.guidance = 'LOS'; % Line-of-Sight guidance
[T_LOS, Y_LOS] = ode45(@(t,y) auv_model_deceleration(t, y, params_LOS), tspan, y0_full, options);

fprintf('Running Carrot Chase Simulation...\n');
params_carrot = common_params;
params_carrot.guidance = 'carrot'; % Carrot Chase guidance
[T_carrot, Y_carrot] = ode45(@(t,y) auv_model_deceleration(t, y, params_carrot), tspan, y0_full, options);

fprintf('Running Non-Linear Guidance (NLG) Simulation...\n');
params_NLG = common_params;
params_NLG.guidance = 'NLG'; % Non-Linear Guidance
params_NLG.K_nlg_p = K_nlg_p;
[T_NLG, Y_NLG] = ode45(@(t,y) auv_model_deceleration(t, y, params_NLG), tspan, y0_full, options);

fprintf('Running True Proportional Navigation (TPN) Simulation...\n');
params_TPN = common_params;
params_TPN.guidance = 'TPN'; % True Proportional Navigation
[T_TPN, Y_TPN] = ode45(@(t,y) auv_model_deceleration(t, y, params_TPN), tspan, y0_full, options);

fprintf('Running Realistic True Proportional Navigation (RTPN) Simulation...\n');
params_RTPN = common_params;
params_RTPN.guidance = 'RTPN'; % Realistic True Proportional Navigation
[T_RTPN, Y_RTPN] = ode45(@(t,y) auv_model_deceleration(t, y, params_RTPN), tspan, y0_full, options);

fprintf('Simulations complete. Plotting results...\n');

% Plot Comparison Results
plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, T_TPN, Y_TPN, T_RTPN, Y_RTPN, target, start, params_carrot);

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

vx_TPN = Y_TPN(:,3) .* cos(Y_TPN(:,4));
vy_TPN = Y_TPN(:,3) .* sin(Y_TPN(:,4));
v_mag_TPN = sqrt(vx_TPN.^2 + vy_TPN.^2);

vx_RTPN = Y_RTPN(:,3) .* cos(Y_RTPN(:,4));
vy_RTPN = Y_RTPN(:,3) .* sin(Y_RTPN(:,4));
v_mag_RTPN = sqrt(vx_RTPN.^2 + vy_RTPN.^2);

% Relative velocity magnitude
subplot(3,1,1);
hold on;
plot(T_LOS, v_mag_LOS, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LOS');
plot(T_carrot, v_mag_carrot, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Carrot');
plot(T_NLG, v_mag_NLG, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'NLG');
plot(T_TPN, v_mag_TPN, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5, 'DisplayName', 'TPN');
plot(T_RTPN, v_mag_RTPN, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5, 'DisplayName', 'RTPN');
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
plot(T_TPN, vx_TPN, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5, 'DisplayName', 'TPN');
plot(T_RTPN, vx_RTPN, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5, 'DisplayName', 'RTPN');
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
plot(T_TPN, vy_TPN, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5, 'DisplayName', 'TPN');
plot(T_RTPN, vy_RTPN, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5, 'DisplayName', 'RTPN');
hold off;
grid on;
xlabel('Time (s)');
ylabel('V_y (m/s)');
title('Y Component of Relative Velocity');
legend;

% Event Function for Docking
% Stops simulation when AUV reaches docking radius
function [value, isterminal, direction] = docking_events(~, y, target, docking_radius)
    % Calculate distance to target
    dist_to_target = sqrt((y(1) - target(1))^2 + (y(2) - target(2))^2);
    value = dist_to_target - docking_radius; % Event occurs when this becomes zero
    isterminal = 1; % Stop the simulation when event occurs
    direction = -1; % Only trigger when value is decreasing (approaching target)
end

% AUV Model with PID Deceleration Control
% Main function that defines the AUV dynamics and control systems
function dydt = auv_model_deceleration(t, y, params)

    % Unpack state vector
    x   = y(1); % Current x position
    y_pos = y(2); % Current y position
    s   = y(3); % Current forward speed
    psi = y(4); % Current heading angle
    integral_heading_error = y(5); % Integral of heading error (for PID)
    previous_heading_error = y(6); % Previous heading error (for derivative term)
    integral_speed_error = y(7); % Integral of speed error (for PID)
    previous_speed_error = y(8); % Previous speed error (for derivative term)
    
    target = params.target; % Target position
    
    % Heading Control 
    if strcmp(params.guidance, 'NLG')
        % Non-Linear Guidance
        rc = nonlinear_guidance(x, y_pos, psi, params);
        heading_error = 0; 
        d_integral_heading_error = 0;
    elseif strcmp(params.guidance, 'TPN') || strcmp(params.guidance, 'RTPN')
        % Hybrid PID + Feed-forward controller for TPN/RTPN
        [rc, heading_error] = tpn_rtpn_guidance(x, y_pos, s, psi, integral_heading_error, params);
        d_integral_heading_error = heading_error;
    else
        % Standard PID controller for LOS and Carrot Chase
        if strcmp(params.guidance, 'carrot')
            desired_psi = carrot_guidance(x, y_pos, params);
        else % Default to LOS
            desired_psi = atan2(target(2) - y_pos, target(1) - x);
        end
        % Calculate heading error (wrapped to [-π, π])
        heading_error = wrapToPi(desired_psi - psi);
        d_integral_heading_error = heading_error;
        % Calculate derivative of heading error
        derivative_heading_error = heading_error - previous_heading_error;
        % PID control law for heading
        rc = params.Kp * heading_error + params.Ki * integral_heading_error + params.Kd * derivative_heading_error;
    end
    
    % Speed Control - manages acceleration/deceleration
    dist_to_target = sqrt((x - target(1))^2 + (y_pos - target(2))^2);
    
    if t < params.accel_duration
        % Phase 1: Initial constant acceleration
        ds = params.initial_a;
        speed_error = 0;
        d_integral_speed_error = 0;
    elseif dist_to_target > params.deceleration_radius
        % Phase 2: Cruising at constant speed
        ds = 0;
        speed_error = 0;
        d_integral_speed_error = 0;
    else
        % Phase 3: Deceleration when close to target
        if dist_to_target <= params.docking_radius
            s_desired = 0; % Stop when within docking radius
        else
            % Gradually reduce speed as approaching target
            dist_in_zone = params.deceleration_radius - params.docking_radius;
            s_desired = params.cruise_speed * sqrt((dist_to_target - params.docking_radius) / dist_in_zone);
        end
        s_desired = max(0, s_desired); % Ensure non-negative speed
        % Calculate speed error
        speed_error = s_desired - s;
        d_integral_speed_error = speed_error;
        derivative_speed_error = speed_error - previous_speed_error;
        % PID control law for speed
        ds = params.Kp_s * speed_error + params.Ki_s * integral_speed_error + params.Kd_s * derivative_speed_error;
    end
    
    % Equations of Motion
    dx = s * cos(psi); % x velocity component
    dy = s * sin(psi); % y velocity component
    dpsi = rc; % Heading rate of change
    
    % Assemble derivative vector for ODE solver
    dydt = [dx; dy; ds; dpsi; d_integral_heading_error; heading_error; d_integral_speed_error; speed_error];
end

% TPN and RTPN Guidance Law Implementation
function [rc, heading_error] = tpn_rtpn_guidance(x, y_pos, s, psi, int_e_psi, params)
    target = params.target;
    N = params.N_gain; % Navigation constant
    L = target - [x; y_pos]; % Vector to target
    R = norm(L); % Distance to target
    if R < 1e-3, rc = 0; heading_error = 0; return; end % Avoid division by zero
    
    % Velocity components
    v_x = s * cos(psi);
    v_y = s * sin(psi);
    
    % Line-of-sight (LOS) angle and rate
    theta = atan2(L(2), L(1));
    theta_dot = (L(1)*v_y - L(2)*v_x) / (R^2);
    
    % Heading error
    heading_error = wrapToPi(theta - psi);
    derivative_term = -theta_dot; % Approximate derivative of heading error
    
    % PID command for heading
    pid_command = params.Kp * heading_error + params.Ki * int_e_psi + params.Kd * derivative_term;
    
    % Relative velocity components
    u_L = L / R; % Unit vector along LOS
    V_R = -u_L' * [v_x; v_y]; % Range rate (negative when closing)
    V_c = -V_R; % Closing velocity (positive when closing)
    
    % Desired lateral acceleration
    if strcmp(params.guidance, 'TPN')
        % TPN
        a_lat_des = N * V_c * theta_dot;
    else % RTPN
        a_lat_des = N * V_R * theta_dot;
    end
    
    % Convert lateral acceleration to heading rate command
    if abs(s) > 0.1
        r_des_feedforward = a_lat_des / s;
    else
        r_des_feedforward = 0; % Avoid division by zero at low speeds
    end
    
    % Combine PID and feedforward commands
    rc = pid_command + r_des_feedforward;
end

% Non-Linear Guidance (Robust Lookahead Version)
function rc = nonlinear_guidance(x, y_pos, psi, params)
    start = params.start;
    target = params.target;
    delta = params.lookahead; % Lookahead distance
    Kp = params.K_nlg_p; % Proportional gain
    
    auv_pos = [x; y_pos];
    path_vec = target - start; % Vector along desired path
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
    
    % Desired course to lookahead point
    desired_course = atan2(carrot_point(2) - y_pos, carrot_point(1) - x);
    course_error = wrapToPi(desired_course - psi);
    
    % Proportional control law
    rc = Kp * course_error;
end

% Carrot Guidance
function desired_psi = carrot_guidance(x, y_pos, params)
    start = params.start;
    target = params.target;
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
    
    % Desired heading to carrot point
    desired_psi = atan2(carrot(2) - y_pos, carrot(1) - x);
end

% Plotting and Helper Functions
function plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, T_TPN, Y_TPN, T_RTPN, Y_RTPN, target, start, params_carrot)
    % Extract parameters
    docking_radius = params_carrot.docking_radius;
    deceleration_radius = params_carrot.deceleration_radius;
    lw = 1.5; % Line width for plots
    
    % Define colors for each guidance method
    nlg_color = [0.8500 0.3250 0.0980]; % Orange
    tpn_color = [0.4940 0.1840 0.5560]; % Purple
    rtpn_color = [0.9290 0.6940 0.1250]; % Yellow
    
    % Extract data from simulation results
    x_LOS = Y_LOS(:,1); y_LOS = Y_LOS(:,2); s_LOS = Y_LOS(:,3); psi_LOS = Y_LOS(:,4);
    x_carrot = Y_carrot(:,1); y_carrot = Y_carrot(:,2); s_carrot = Y_carrot(:,3); psi_carrot = Y_carrot(:,4);
    x_NLG = Y_NLG(:,1); y_NLG = Y_NLG(:,2); s_NLG = Y_NLG(:,3); psi_NLG = Y_NLG(:,4);
    x_TPN = Y_TPN(:,1); y_TPN = Y_TPN(:,2); s_TPN = Y_TPN(:,3); psi_TPN = Y_TPN(:,4);
    x_RTPN = Y_RTPN(:,1); y_RTPN = Y_RTPN(:,2); s_RTPN = Y_RTPN(:,3); psi_RTPN = Y_RTPN(:,4);
    





    % Figure 1: Trajectory Comparison
    figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 800, 600]);
    hold on;
    % Plot trajectories for each guidance method
    plot(x_LOS, y_LOS, 'r-', 'LineWidth', lw, 'DisplayName', 'LOS');
    plot(x_carrot, y_carrot, 'g-', 'LineWidth', lw, 'DisplayName', 'Carrot');
    plot(x_NLG, y_NLG, 'Color', nlg_color, 'LineWidth', lw, 'DisplayName', 'NLG');
    plot(x_TPN, y_TPN, 'Color', tpn_color, 'LineWidth', lw, 'DisplayName', 'TPN');
    plot(x_RTPN, y_RTPN, 'Color', rtpn_color, 'LineWidth', lw, 'DisplayName', 'RTPN');
    
    % Plot reference elements
    plot([start(1), target(1)], [start(2), target(2)], 'k--', 'LineWidth', 1.0, 'DisplayName', 'Desired Path');
    plot(target(1), target(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Target');
    plot(start(1), start(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'Start');
    
    % Plot docking and deceleration zones
    theta_circle = linspace(0, 2*pi, 100);
    plot(target(1) + docking_radius*cos(theta_circle), target(2) + docking_radius*sin(theta_circle), 'c--', 'LineWidth', 1.0, 'DisplayName', 'Docking Radius');
    plot(target(1) + deceleration_radius*cos(theta_circle), target(2) + deceleration_radius*sin(theta_circle), '-.', 'Color', '#999900', 'LineWidth', 1.0, 'DisplayName', 'Decel. Zone');
    
    hold off;
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('AUV Trajectory Comparison');
    legend('Location', 'best');
    





    % Figure 2: Performance Metrics Comparison
    figure('Name', 'Performance Metrics Comparison', 'Position', [950, 100, 800, 700]);
    
    % Distance to target for each method
    dist_error_LOS = sqrt((x_LOS - target(1)).^2 + (y_LOS - target(2)).^2);
    dist_error_carrot = sqrt((x_carrot - target(1)).^2 + (y_carrot - target(2)).^2);
    dist_error_NLG = sqrt((x_NLG - target(1)).^2 + (y_NLG - target(2)).^2);
    dist_error_TPN = sqrt((x_TPN - target(1)).^2 + (y_TPN - target(2)).^2);
    dist_error_RTPN = sqrt((x_RTPN - target(1)).^2 + (y_RTPN - target(2)).^2);
    
    % Distance to target over time
    subplot(2,2,1);
    hold on;
    plot(T_LOS, dist_error_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, dist_error_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, dist_error_NLG, 'Color', nlg_color, 'LineWidth', lw);
    plot(T_TPN, dist_error_TPN, 'Color', tpn_color, 'LineWidth', lw);
    plot(T_RTPN, dist_error_RTPN, 'Color', rtpn_color, 'LineWidth', lw);
    
    % Add reference lines for docking and deceleration radii
    yline(docking_radius, 'c--', 'Docking Radius');
    yline(deceleration_radius, 'y-.', 'Decel. Zone');
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Distance to Target');
    legend('LOS', 'Carrot', 'NLG', 'TPN', 'RTPN', 'Location', 'northeast');
    
    % Heading error comparison
    subplot(2,2,2);
    hold on;
    % Heading error for each method
    plot(T_LOS, rad2deg(wrapToPi(atan2(target(2) - y_LOS, target(1) - x_LOS) - psi_LOS)), 'r-', 'LineWidth', lw);
    
    % For carrot guidance, need to calculate desired heading at each point
    desired_psi_carrot = zeros(size(x_carrot));
    for i = 1:length(x_carrot)
        desired_psi_carrot(i) = carrot_guidance(x_carrot(i), y_carrot(i), params_carrot);
    end
    plot(T_carrot, rad2deg(wrapToPi(desired_psi_carrot - psi_carrot)), 'g-', 'LineWidth', lw);
    
    plot(T_NLG, rad2deg(wrapToPi(atan2(target(2) - y_NLG, target(1) - x_NLG) - psi_NLG)), 'Color', nlg_color, 'LineWidth', lw);
    plot(T_TPN, rad2deg(wrapToPi(atan2(target(2) - y_TPN, target(1) - x_TPN) - psi_TPN)), 'Color', tpn_color, 'LineWidth', lw);
    plot(T_RTPN, rad2deg(wrapToPi(atan2(target(2) - y_RTPN, target(1) - x_RTPN) - psi_RTPN)), 'Color', rtpn_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Heading Error (deg)');
    title('Heading Error Comparison');
    ylim([-190, 190]); % Limit y-axis to reasonable range
    
    % Path following performance (cross-track error)
    subplot(2,2,3);
    hold on;
    % Cross-track error for each method
    plot(T_LOS, calculate_cross_track(x_LOS, y_LOS, start, target), 'r-', 'LineWidth', lw);
    plot(T_carrot, calculate_cross_track(x_carrot, y_carrot, start, target), 'g-', 'LineWidth', lw);
    plot(T_NLG, calculate_cross_track(x_NLG, y_NLG, start, target), 'Color', nlg_color, 'LineWidth', lw);
    plot(T_TPN, calculate_cross_track(x_TPN, y_TPN, start, target), 'Color', tpn_color, 'LineWidth', lw);
    plot(T_RTPN, calculate_cross_track(x_RTPN, y_RTPN, start, target), 'Color', rtpn_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Path Following Performance');
    
    % Control effort (turn rate)
    subplot(2,2,4);
    hold on;
    % Turn rate for each method (derivative of heading)
    plot(T_LOS, rad2deg([0; diff(Y_LOS(:,4)) ./ diff(T_LOS)]), 'r-', 'LineWidth', lw);
    plot(T_carrot, rad2deg([0; diff(Y_carrot(:,4)) ./ diff(T_carrot)]), 'g-', 'LineWidth', lw);
    plot(T_NLG, rad2deg([0; diff(Y_NLG(:,4)) ./ diff(T_NLG)]), 'Color', nlg_color, 'LineWidth', lw);
    plot(T_TPN, rad2deg([0; diff(Y_TPN(:,4)) ./ diff(T_TPN)]), 'Color', tpn_color, 'LineWidth', lw);
    plot(T_RTPN, rad2deg([0; diff(Y_RTPN(:,4)) ./ diff(T_RTPN)]), 'Color', rtpn_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Turn Rate (deg/s)');
    title('Control Effort Comparison');
    
    % Add overall title to the figure
    sgtitle('AUV Guidance: Core Performance Metrics', 'FontSize', 14, 'FontWeight', 'bold');
    





    % Figure 3: Speed and Efficiency Metrics
    figure('Name', 'Speed and Efficiency Metrics', 'Position', [100, 50, 900, 700]);
    
    % Create categorical array for guidance methods
    labels = categorical({'LOS', 'Carrot', 'NLG', 'TPN', 'RTPN'});
    labels = reordercats(labels, {'LOS', 'Carrot', 'NLG', 'TPN', 'RTPN'});
    
    % Extract simulation end times and path lengths
    times = [T_LOS(end), T_carrot(end), T_NLG(end), T_TPN(end), T_RTPN(end)];
    path_lengths = [calculate_path_length(x_LOS, y_LOS), ...
                   calculate_path_length(x_carrot, y_carrot), ...
                   calculate_path_length(x_NLG, y_NLG), ...
                   calculate_path_length(x_TPN, y_TPN), ...
                   calculate_path_length(x_RTPN, y_RTPN)];
    
    % Speed profiles
    subplot(2,2,[1,2]);
    hold on;
    plot(T_LOS, s_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, s_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, s_NLG, 'Color', nlg_color, 'LineWidth', lw);
    plot(T_TPN, s_TPN, 'Color', tpn_color, 'LineWidth', lw);
    plot(T_RTPN, s_RTPN, 'Color', rtpn_color, 'LineWidth', lw);
    grid on; hold off;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('AUV Speed Profile');
    legend('LOS', 'Carrot', 'NLG', 'TPN', 'RTPN', 'Location', 'best');
    
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
    yline(norm(target - start), 'r--', 'Min. Distance');
    
    % Overall title to the figure
    sgtitle('AUV Guidance: Speed and Efficiency', 'FontSize', 14, 'FontWeight', 'bold');
    





    % Figure 4: Deceleration Performance Analysis
    figure('Name', 'Deceleration Performance Analysis', 'Position', [950, 50, 900, 400]);
    
    % Final speeds and distances
    final_speeds = [s_LOS(end), s_carrot(end), s_NLG(end), s_TPN(end), s_RTPN(end)];
    final_distances = [dist_error_LOS(end), dist_error_carrot(end), dist_error_NLG(end), dist_error_TPN(end), dist_error_RTPN(end)];
    
    % Find when deceleration starts for each method
    all_dist_errors = {dist_error_LOS, dist_error_carrot, dist_error_NLG, dist_error_TPN, dist_error_RTPN};
    all_times = {T_LOS, T_carrot, T_NLG, T_TPN, T_RTPN};
    decel_start_times = NaN(1, 5); % Initialize with NaN
    
    for i = 1:5
        idx = find(all_dist_errors{i} <= deceleration_radius, 1, 'first');
        if ~isempty(idx)
            decel_start_times(i) = all_times{i}(idx);
        end
    end
    
    % Final approach performance (speed vs distance)
    subplot(1, 2, 1);
    markers = {'o', 's', 'p', 'd', '^'}; % Different markers for each method
    colors = {'r', 'g', nlg_color, tpn_color, rtpn_color}; % Colors for each method
    
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
    title('Final Approach Performance');
    xlabel('Final Distance to Target (m)');
    ylabel('Final Speed (m/s)');
    legend('show', 'Location', 'northeast');

    % Reference line for docking radius
    xline(docking_radius, 'k--', 'Target Radius');
    
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
    sgtitle('AUV Guidance: Deceleration Analysis', 'FontSize', 14, 'FontWeight', 'bold');
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
    
    cross_track = zeros(size(x_pos));
    for i = 1:length(x_pos)
        % Vector from start to current position
        pos_vec = [x_pos(i); y_pos(i)] - start;
        % Cross-track error is the absolute dot product with normal vector
        cross_track(i) = abs(dot(pos_vec, normal_vec));
    end
end
