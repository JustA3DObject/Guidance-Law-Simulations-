% AUV Initial Conditions
x0 = -100;     % Initial x position (m)
y0 = -100;     % Initial y position (m)
psi0 = deg2rad(-90); % Initial heading (rad)
start = [x0; y0];

% Docking Target Point
xt = 0;        % Target x position (m)
yt = 0;        % Target y position (m)
target = [xt; yt];

% AUV Motion Profile Parameters
initial_v = 1.0;      % Initial forward speed at t=0 (m/s)
initial_a = 0;      % Constant acceleration for the initial phase (m/s^2)
accel_duration = 0; % Duration of the initial acceleration phase (s)
cruise_speed = initial_v + initial_a * accel_duration; % Final cruise speed

% AUV and Controller Parameters
% Heading PID Controller
Kp = 1.5;      % Proportional gain
Ki = 0.05;     % Integral gain
Kd = 0.03;     % Derivative gain

% Speed PID Controller (for deceleration)
Kp_s = 3.0;    % Proportional gain for speed
Ki_s = 0.1;    % Integral gain for speed
Kd_s = 0.05;   % Derivative gain for speed

% Guidance Parameters
docking_radius = 5.0; % Radius for successful docking (m)
deceleration_radius = 40.0; % Radius at which to start decelerating (m)
lookahead = 1.0; % Lookahead distance for Carrot Chase and NLG (m)
K_nlg_p = 1.0;    % Proportional gain for the robust NLG heading controller

% Simulation Time
tspan = [0 200]; % Simulation time span (s)

% Initial State Vector
% State: [x; y; s; psi; integral_heading_error; previous_heading_error; integral_speed_error; previous_speed_error]
y0_full = [x0; y0; initial_v; psi0; 0; 0; 0; 0];

% Simulation Setup
options = odeset('Events', @(t,y) docking_events(t, y, target, docking_radius));

% Common parameters to be passed to the model
common_params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
                       'Kp_s', Kp_s, 'Ki_s', Ki_s, 'Kd_s', Kd_s, ...
                       'target', target, 'docking_radius', docking_radius, ...
                       'deceleration_radius', deceleration_radius, ...
                       'cruise_speed', cruise_speed, ...
                       'start', start, 'lookahead', lookahead, ...
                       'initial_a', initial_a, 'accel_duration', accel_duration);
fprintf('Running LOS Guidance Simulation...\n');
params_LOS = common_params;
params_LOS.guidance = 'LOS';
[T_LOS, Y_LOS] = ode45(@(t,y) auv_model_deceleration(t, y, params_LOS), tspan, y0_full, options);
fprintf('Running Carrot Chase Simulation...\n');
params_carrot = common_params;
params_carrot.guidance = 'carrot';
[T_carrot, Y_carrot] = ode45(@(t,y) auv_model_deceleration(t, y, params_carrot), tspan, y0_full, options);
fprintf('Running Non-Linear Guidance (NLG) Simulation...\n');
params_NLG = common_params;
params_NLG.guidance = 'NLG';
params_NLG.K_nlg_p = K_nlg_p;
[T_NLG, Y_NLG] = ode45(@(t,y) auv_model_deceleration(t, y, params_NLG), tspan, y0_full, options);
fprintf('Simulations complete. Plotting results...\n');

% Plot Comparison Results
plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, target, start, params_carrot);

% Event Function for Docking
function [value, isterminal, direction] = docking_events(~, y, target, docking_radius)
    % This function defines the event that stops the simulation.
    % It triggers when the AUV's distance to the target is equal to the docking radius.
    dist_to_target = sqrt((y(1) - target(1))^2 + (y(2) - target(2))^2);
    value = dist_to_target - docking_radius; % Stop when distance equals docking_radius
    isterminal = 1; % Terminate the simulation
    direction = -1; % Event triggers when value is decreasing
end

% AUV Model with PID Deceleration Control
function dydt = auv_model_deceleration(t, y, params)
    % Unpack state vector (8 elements)
    x   = y(1);
    y_pos = y(2);
    s   = y(3); % Current forward speed
    psi = y(4);
    integral_heading_error = y(5);
    previous_heading_error = y(6);
    integral_speed_error = y(7);
    previous_speed_error = y(8);
    
    target = params.target;
    
    % Heading Control
    % Select guidance law to determine the desired heading (psi)
    if strcmp(params.guidance, 'NLG')
        rc = nonlinear_guidance(x, y_pos, psi, params);
        heading_error = 0; d_integral_heading_error = 0;
    else
        % PID controller for other guidance laws (LOS, Carrot)
        if strcmp(params.guidance, 'carrot')
            desired_psi = carrot_guidance(x, y_pos, params);
        else % Default to LOS
            desired_psi = atan2(target(2) - y_pos, target(1) - x);
        end
        heading_error = wrapToPi(desired_psi - psi);
        d_integral_heading_error = heading_error;
        derivative_heading_error = heading_error - previous_heading_error;
        rc = params.Kp * heading_error + params.Ki * integral_heading_error + params.Kd * derivative_heading_error;
    end
    
    % Speed Control 
    dist_to_target = sqrt((x - target(1))^2 + (y_pos - target(2))^2);
    
    if t < params.accel_duration
        % Phase 1: Initial constant acceleration
        ds = params.initial_a;
        speed_error = 0;
        d_integral_speed_error = 0;
    elseif dist_to_target > params.deceleration_radius
        % Phase 2: Cruising at constant speed (after initial acceleration)
        ds = 0;
        speed_error = 0;
        d_integral_speed_error = 0;
    else
        % Phase 3: Deceleration using PID controller
        % This phase starts when the AUV enters the deceleration_radius.
        
        % For docking, calculate desired speed based on distance to target.
        % The desired speed decreases to 0 as the AUV approaches the docking radius.
        if dist_to_target <= params.docking_radius
            s_desired = 0;
        else
            % Scale speed based on remaining distance within the deceleration zone
            dist_in_zone = params.deceleration_radius - params.docking_radius;
            s_desired = params.cruise_speed * sqrt((dist_to_target - params.docking_radius) / dist_in_zone);
        end
        
        % Clamp desired speed to be non-negative
        s_desired = max(0, s_desired);
        
        % Speed PID Controller
        speed_error = s_desired - s;
        d_integral_speed_error = speed_error;
        derivative_speed_error = speed_error - previous_speed_error;
        
        ds = params.Kp_s * speed_error + params.Ki_s * integral_speed_error + params.Kd_s * derivative_speed_error;
    end
    
    % Equations of Motion
    dx = s * cos(psi);
    dy = s * sin(psi);
    dpsi = rc;
    
    % Assemble the derivatives vector for the ODE solver
    dydt = [dx; dy; ds; dpsi; d_integral_heading_error; heading_error; d_integral_speed_error; speed_error];
end
% Guidance Law Functions

% Non-Linear Guidance (Robust Lookahead Version)
function rc = nonlinear_guidance(x, y_pos, psi, params)
    start = params.start;
    target = params.target;
    delta = params.lookahead; % Lookahead distance
    Kp = params.K_nlg_p;      % Proportional gain for heading control
    
    auv_pos = [x; y_pos];
    path_vec = target - start;
    carrot_point = target; % Default carrot to the end of the path
    
    if dot(path_vec, path_vec) > 1e-10
        projection_ratio = dot(auv_pos - start, path_vec) / dot(path_vec, path_vec);
        projection_point = start + max(0, min(1, projection_ratio)) * path_vec;
        dist_remaining = norm(target - projection_point);
        lookahead_dist = min(delta, dist_remaining);
        if dist_remaining > 1e-6
            carrot_point = projection_point + lookahead_dist * (path_vec / norm(path_vec));
        end
    end
    desired_course = atan2(carrot_point(2) - y_pos, carrot_point(1) - x);
    course_error = wrapToPi(desired_course - psi);
    rc = Kp * course_error;
end

% Carrot Guidance
function desired_psi = carrot_guidance(x, y_pos, params)
    start=params.start; target=params.target; L=params.lookahead;
    P=[x;y_pos]; A=start; B=target; V=B-A;
    if dot(V,V)<1e-10, carrot=B;
    else
        Q = A + max(0,min(1,dot(P-A,V)/dot(V,V)))*V;
        to_target=B-Q; d=norm(to_target);
        if d>0, carrot=Q+min(L,d)*(to_target/d); else, carrot=Q; end
    end
    desired_psi = atan2(carrot(2)-y_pos, carrot(1)-x);
end

% Plotting Function
function plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, target, start, params_carrot)
    docking_radius = params_carrot.docking_radius;
    deceleration_radius = params_carrot.deceleration_radius;
    lw = 1.5; % Define a common LineWidth for visibility
    nlg_color = [0.8500 0.3250 0.0980]; % Define NLG color for consistency
    % Extract Data for all algorithms
    x_LOS = Y_LOS(:,1); y_LOS = Y_LOS(:,2); s_LOS = Y_LOS(:,3); psi_LOS = Y_LOS(:,4);
    x_carrot = Y_carrot(:,1); y_carrot = Y_carrot(:,2); s_carrot = Y_carrot(:,3); psi_carrot = Y_carrot(:,4);
    x_NLG = Y_NLG(:,1); y_NLG = Y_NLG(:,2); s_NLG = Y_NLG(:,3); psi_NLG = Y_NLG(:,4);
    
    % FIGURE 1: Trajectory Comparison
    figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 800, 600]);
    hold on;
    plot(x_LOS, y_LOS, 'r-', 'LineWidth', lw, 'DisplayName', 'LOS Guidance');
    plot(x_carrot, y_carrot, 'g-', 'LineWidth', lw, 'DisplayName', 'Carrot Chase');
    plot(x_NLG, y_NLG, 'Color', nlg_color, 'LineWidth', lw, 'DisplayName', 'Non-Linear Guidance (NLG)');
    plot([start(1), target(1)], [start(2), target(2)], 'k--', 'LineWidth', 1.0, 'DisplayName', 'Desired Path');
    plot(target(1), target(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Target');
    plot(start(1), start(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'Start');
    theta_circle = linspace(0, 2*pi, 100);
    plot(target(1) + docking_radius*cos(theta_circle), target(2) + docking_radius*sin(theta_circle), 'c--', 'LineWidth', 1.0, 'DisplayName', 'Docking Radius');
    plot(target(1) + deceleration_radius*cos(theta_circle), target(2) + deceleration_radius*sin(theta_circle), '-.', 'Color', '#999900', 'LineWidth', 1.0, 'DisplayName', 'Deceleration Zone');
    hold off;
    axis equal; grid on;
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    title('AUV Trajectory Comparison');
    legend('Location', 'best');
    
    % FIGURE 2: Performance Metrics Comparison
    figure('Name', 'Performance Metrics Comparison', 'Position', [950, 100, 800, 700]);
    
    % Distance to Target
    subplot(2,2,1);
    hold on;
    dist_error_LOS = sqrt((x_LOS - target(1)).^2 + (y_LOS - target(2)).^2);
    dist_error_carrot = sqrt((x_carrot - target(1)).^2 + (y_carrot - target(2)).^2);
    dist_error_NLG = sqrt((x_NLG - target(1)).^2 + (y_NLG - target(2)).^2);
    plot(T_LOS, dist_error_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, dist_error_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, dist_error_NLG, 'Color', nlg_color, 'LineWidth', lw);
    yline(docking_radius, 'c--', 'Docking Radius');
    yline(deceleration_radius, 'y-.', 'Decel. Zone');
    hold off; grid on; xlabel('Time (s)'); ylabel('Distance (m)');
    title('Distance to Target/Center');
    legend('LOS', 'Carrot', 'NLG', 'Location', 'northeast');
    % Heading Error Comparison
    subplot(2,2,2);
    hold on;
    desired_psi_LOS = atan2(target(2) - y_LOS, target(1) - x_LOS);
    heading_error_LOS = wrapToPi(desired_psi_LOS - psi_LOS);
    desired_psi_carrot = zeros(size(x_carrot));
    for i=1:length(x_carrot), desired_psi_carrot(i) = carrot_guidance(x_carrot(i), y_carrot(i), params_carrot); end
    heading_error_carrot = wrapToPi(desired_psi_carrot - psi_carrot);
    desired_psi_NLG = atan2(target(2) - y_NLG, target(1) - x_NLG);
    heading_error_NLG = wrapToPi(desired_psi_NLG - psi_NLG);
    plot(T_LOS, rad2deg(heading_error_LOS), 'r-', 'LineWidth', lw);
    plot(T_carrot, rad2deg(heading_error_carrot), 'g-', 'LineWidth', lw);
    plot(T_NLG, rad2deg(heading_error_NLG), 'Color', nlg_color, 'LineWidth', lw);
    hold off; grid on; xlabel('Time (s)'); ylabel('Heading Error (deg)');
    title('Heading Error Comparison'); ylim([-190, 190]);
    
    % Path Following Performance
    subplot(2,2,3);
    hold on;
    cross_track_LOS = calculate_cross_track(x_LOS, y_LOS, start, target);
    cross_track_carrot = calculate_cross_track(x_carrot, y_carrot, start, target);
    cross_track_NLG = calculate_cross_track(x_NLG, y_NLG, start, target);
    plot(T_LOS, cross_track_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, cross_track_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, cross_track_NLG, 'Color', nlg_color, 'LineWidth', lw);
    hold off; grid on; xlabel('Time (s)'); ylabel('Error (m)');
    title('Path Following Performance');
    
    % Control Effort Comparison
    subplot(2,2,4);
    hold on;
    turn_rate_LOS = [0; diff(Y_LOS(:,4)) ./ diff(T_LOS)];
    turn_rate_carrot = [0; diff(Y_carrot(:,4)) ./ diff(T_carrot)];
    turn_rate_NLG = [0; diff(Y_NLG(:,4)) ./ diff(T_NLG)];
    plot(T_LOS, rad2deg(turn_rate_LOS), 'r-', 'LineWidth', lw);
    plot(T_carrot, rad2deg(turn_rate_carrot), 'g-', 'LineWidth', lw);
    plot(T_NLG, rad2deg(turn_rate_NLG), 'Color', nlg_color, 'LineWidth', lw);
    hold off; grid on; xlabel('Time (s)'); ylabel('Turn Rate (deg/s)');
    title('Control Effort Comparison');
    sgtitle('AUV Guidance: Core Performance Metrics', 'FontSize', 14, 'FontWeight', 'bold');
    
    % FIGURE 3: Speed and Efficiency Metrics
    figure('Name', 'Speed and Efficiency Metrics', 'Position', [100, 50, 900, 700]);
    
    % Speed Profile
    subplot(2,2,[1,2]); 
    hold on;
    plot(T_LOS, s_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, s_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, s_NLG, 'Color', nlg_color, 'LineWidth', lw);
    grid on; hold off;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('AUV Speed Profile');
    legend('LOS', 'Carrot', 'NLG', 'Location', 'best');
    % Calculate Metrics for bar charts
    labels = categorical({'LOS', 'Carrot', 'NLG'});
    labels = reordercats(labels, {'LOS', 'Carrot', 'NLG'});
    times = [T_LOS(end), T_carrot(end), T_NLG(end)];
    path_lengths = [
        calculate_path_length(x_LOS, y_LOS), ...
        calculate_path_length(x_carrot, y_carrot), ...
        calculate_path_length(x_NLG, y_NLG)
    ];
    
    % Time to Target Bar Chart
    subplot(2,2,3);
    b1 = bar(labels, times);
    grid on; title('Time to Reach Target'); ylabel('Time (s)');
    text(b1.XEndPoints, b1.YEndPoints, string(round(b1.YData, 1)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
    
    % Path Length Bar Chart
    subplot(2,2,4);
    b2 = bar(labels, path_lengths);
    grid on; title('Total Path Length'); ylabel('Distance (m)');
    text(b2.XEndPoints, b2.YEndPoints, string(round(b2.YData, 1)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
    min_dist = norm(target - start);
    yline(min_dist, 'r--', 'Min. Distance');
    
    sgtitle('AUV Guidance: Speed and Efficiency', 'FontSize', 14, 'FontWeight', 'bold');
    % FIGURE 4: Deceleration Performance Analysis
    figure('Name', 'Deceleration Performance Analysis', 'Position', [950, 50, 900, 400]);
    
    % Data Extraction for Deceleration Analysis
    % Final Speeds at docking radius
    final_speeds = [s_LOS(end), s_carrot(end), s_NLG(end)];
    % Final Distances at docking radius
    final_distances = [dist_error_LOS(end), dist_error_carrot(end), dist_error_NLG(end)];
    
    % Find time when deceleration starts for each method
    idx_decel_LOS = find(dist_error_LOS <= deceleration_radius, 1, 'first');
    time_decel_LOS = T_LOS(idx_decel_LOS);
    
    idx_decel_carrot = find(dist_error_carrot <= deceleration_radius, 1, 'first');
    time_decel_carrot = T_carrot(idx_decel_carrot);
    
    idx_decel_NLG = find(dist_error_NLG <= deceleration_radius, 1, 'first');
    time_decel_NLG = T_NLG(idx_decel_NLG);
    
    decel_start_times = [time_decel_LOS, time_decel_carrot, time_decel_NLG];
    % Final Approach Performance (Speed vs. Distance)
    subplot(1, 2, 1);
    markers = {'o', 's', 'p'};
    colors = {'r', 'g', nlg_color};
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
    % Add a reference line for the target docking radius
    xline(docking_radius, 'k--', 'Target Radius');
    % Deceleration Start Time
    subplot(1, 2, 2);
    b_decel = bar(labels, decel_start_times);
    grid on;
    title('Deceleration Start Time');
    ylabel('Time (s)');
    text(b_decel.XEndPoints, b_decel.YEndPoints, string(round(b_decel.YData, 1)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
    sgtitle('AUV Guidance: Deceleration Analysis', 'FontSize', 14, 'FontWeight', 'bold');
end
% Helper Functions
function wrapped = wrapToPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end
function len = calculate_path_length(x_coords, y_coords)
    if numel(x_coords) < 2, len = 0; return; end
    len = sum(sqrt(diff(x_coords).^2 + diff(y_coords).^2));
end
function cross_track = calculate_cross_track(x_pos, y_pos, start, target)
    path_vec = target - start;
    path_length = norm(path_vec);
    if path_length < 1e-10, cross_track = norm([x_pos;y_pos]-start); return; end
    path_unit = path_vec / path_length;
    normal_vec = [-path_unit(2); path_unit(1)];
    cross_track = zeros(size(x_pos));
    for i = 1:length(x_pos)
        pos_vec = [x_pos(i); y_pos(i)] - start;
        cross_track(i) = abs(dot(pos_vec, normal_vec));
    end
end
