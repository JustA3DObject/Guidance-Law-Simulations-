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
v_dock = [dock_vx; dock_vy];

% AUV Motion Profile Parameters
initial_v = 0;       % Initial forward speed at t=0 (m/s)
initial_a = 0.8;       % Constant acceleration for the initial phase (m/s^2)
accel_duration = 5; % Duration of the initial acceleration phase (s)
% Calculate final cruise speed after acceleration phase
cruise_speed = initial_v + initial_a * accel_duration;

% Log-Polynomial (3-degree) Deceleration Parameters
poly_a = 2.0; % Polynomial coefficient a
poly_b = 2.0; % Polynomial coefficient b
poly_c = 1.0; % Polynomial coefficient c
initial_distance_ref = norm(start - dock_start_pos); % Reference distance for normalization

% AUV and Controller Parameters
% Heading PID Controller - controls the AUV's direction
Kp = 1.5;       % Proportional gain - responds to current error
Ki = 0.05;      % Integral gain - eliminates steady-state error
Kd = 0.03;      % Derivative gain - dampens oscillations

% Guidance Parameters
N_gain = 4.0;  % Proportional Navigation constant for TPN/RTPN guidance laws
docking_radius = 0.75; % Radius for successful docking (m) - considered "docked" within this distance
lookahead = 1.0; % Lookahead distance for Carrot Chase and NLG (m) - how far ahead to "look"
K_nlg_p = 1.0;     % Proportional gain for the robust NLG heading controller

% Simulation Time
tspan = [0 500]; % Simulation time span (s)

% --- ADDITIONS FOR WATER CURRENT EFFECTS ---
% Current velocity
v_current_magnitude = 0.5; % Magnitude of water current (m/s)
% The current direction is perpendicular to the shortest path from start to the initial dock position.
shortest_path_vector = dock_start_pos - start;
shortest_path_angle = atan2(shortest_path_vector(2), shortest_path_vector(1));
current_angle = shortest_path_angle + deg2rad(90);
% Calculate current velocity components
vx_current = v_current_magnitude * cos(current_angle);
vy_current = v_current_magnitude * sin(current_angle);
v_current = [vx_current; vy_current];

% New final velocity at docking
% The AUV's final ground velocity should match the dock's velocity.
% AUV_ground_v_final = v_dock
% AUV_rel_water_v_final + v_current = v_dock
% AUV_rel_water_v_final = v_dock - v_current
final_velocity = v_dock - v_current;
final_speed = norm(final_velocity); % Final magnitude of AUV's speed relative to water

% --------------------------------------------
% Initial State Vector
% State: [x; y; s; psi; integral_heading_error; previous_heading_error]
y0_full = [x0; y0; initial_v; psi0; 0; 0];

% Simulation Setup
% Configure ODE solver to stop when docking event occurs
options = odeset('Events', @(t,y) docking_events(t, y, dock_start_pos, v_dock, docking_radius));

% Common parameters to be passed to the model
common_params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
                       'dock_start_pos', dock_start_pos, ...
                       'v_dock', v_dock, ...
                       'docking_radius', docking_radius, ...
                       'cruise_speed', cruise_speed, ...
                       'start', start, 'lookahead', lookahead, ...
                       'initial_a', initial_a, 'accel_duration', accel_duration, ...
                       'N_gain', N_gain, 'poly_a', poly_a, 'poly_b', poly_b, ...
                       'poly_c', poly_c, 'initial_distance_ref', initial_distance_ref, ...
                       'v_current', v_current, ...
                       'final_speed', final_speed);

% Run simulations for each guidance algorithm
fprintf('Running LOS Guidance Simulation...\n');
params_LOS = common_params;
params_LOS.guidance = 'LOS'; % Line-of-Sight guidance
[T_LOS, Y_LOS] = ode45(@(t,y) auv_model_log_poly_decel(t, y, params_LOS), tspan, y0_full, options);

fprintf('Running Carrot Chase Simulation...\n');
params_carrot = common_params;
params_carrot.guidance = 'carrot'; % Carrot Chase guidance
[T_carrot, Y_carrot] = ode45(@(t,y) auv_model_log_poly_decel(t, y, params_carrot), tspan, y0_full, options);

fprintf('Running Non-Linear Guidance (NLG) Simulation...\n');
params_NLG = common_params;
params_NLG.guidance = 'NLG'; % Non-Linear Guidance
params_NLG.K_nlg_p = K_nlg_p;
[T_NLG, Y_NLG] = ode45(@(t,y) auv_model_log_poly_decel(t, y, params_NLG), tspan, y0_full, options);

fprintf('Simulations complete. Plotting results...\n');

% Plot Comparison Results
plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, ...
                dock_start_pos, v_dock, start, params_carrot);

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
title('AUV Velocity Magnitude Relative to Water');
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
title('X Component of AUV Velocity Relative to Water');
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
title('Y Component of AUV Velocity Relative to Water');
legend;

% Figure 6: Range Rate (rdot) to Target
figure('Name', 'Range Rate (rdot)', 'Position', [100, 100, 1200, 600]);
% Helper function for rdot calculation
calculate_rdot = @(Y, dock_start_pos, v_dock, v_current, T) ...
-( ...
((dock_start_pos(1) + v_dock(1)*T - Y(:,1)) .* (Y(:,3) .* cos(Y(:,4)) + v_current(1))) + ...
((dock_start_pos(2) + v_dock(2)*T - Y(:,2)) .* (Y(:,3) .* sin(Y(:,4)) + v_current(2))) ...
) ./ sqrt((dock_start_pos(1) + v_dock(1)*T - Y(:,1)).^2 + (dock_start_pos(2) + v_dock(2)*T - Y(:,2)).^2);

% rdot for each guidance law
rdot_LOS = calculate_rdot(Y_LOS, dock_start_pos, v_dock, v_current, T_LOS);
rdot_carrot = calculate_rdot(Y_carrot, dock_start_pos, v_dock, v_current, T_carrot);
rdot_NLG = calculate_rdot(Y_NLG, dock_start_pos, v_dock, v_current, T_NLG);

% Plot the results
hold on;
plot(T_LOS, rdot_LOS, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LOS');
plot(T_carrot, rdot_carrot, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Carrot');
plot(T_NLG, rdot_NLG, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'NLG');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Range Rate (m/s)');
title('Range Rate (Closing Velocity) to Moving Dock');
legend('Location', 'southeast');
yline(0, 'k--', 'DisplayName', 'Zero Closing Velocity');

% Event Function for Docking
function [value, isterminal, direction] = docking_events(t, y, dock_start_pos, v_dock, docking_radius)
    % Calculate current target position
    target = dock_start_pos + v_dock * t;
    % Calculate distance to target
    dist_to_target = norm(y(1:2) - target);
    value = dist_to_target - docking_radius; % Event occurs when this becomes zero
    isterminal = 1; % Stop the simulation when event occurs
    direction = -1; % Only trigger when value is decreasing (approaching target)
end

% AUV Model with Log Polynomial Deceleration Control
function dydt = auv_model_log_poly_decel(t, y, params)
    % Unpack state vector
    x   = y(1); % Current x position
    y_pos = y(2); % Current y position
    s   = y(3); % Current forward speed (relative to water)
    psi = y(4); % Current heading angle
    integral_heading_error = y(5); % Integral of heading error (for PID)
    previous_heading_error = y(6); % Previous heading error (for derivative term)
    
    % Calculate the current position of the moving target
    target = params.dock_start_pos + params.v_dock * t;
    
    % Heading Control 
    if strcmp(params.guidance, 'NLG')
        % Non-Linear Guidance
        rc = nonlinear_guidance(x, y_pos, psi, s, params, target);
        heading_error = 0; 
        d_integral_heading_error = 0;
    else
        % Standard PID controller for LOS and Carrot Chase
        if strcmp(params.guidance, 'carrot')
            desired_psi = carrot_guidance(x, y_pos, s, params, target);
        else % Default to LOS
            % Calculate desired heading to counteract current
            v_ground_des_vec = target - [x; y_pos];
            if norm(v_ground_des_vec) > 1e-6
                v_ground_des_unit = v_ground_des_vec / norm(v_ground_des_vec);
                v_auv_rel_water_des_vec = (s * v_ground_des_unit) - params.v_current;
                desired_psi = atan2(v_auv_rel_water_des_vec(2), v_auv_rel_water_des_vec(1));
            else
                desired_psi = psi;
            end
        end
        % Calculate heading error (wrapped to [-pi, pi])
        heading_error = wrapToPi(desired_psi - psi);
        d_integral_heading_error = heading_error;
        % Calculate derivative of heading error
        derivative_heading_error = heading_error - previous_heading_error;
        % PID control law for heading
        rc = params.Kp * heading_error + params.Ki * integral_heading_error + params.Kd * derivative_heading_error;
    end
    
    % Limit the maximum angle for correcting the heading angle
    max_correction_rad = deg2rad(30);
    rc = max(-max_correction_rad, min(max_correction_rad, rc));
    
    % Speed Control - Log Polynomial Law for Deceleration
    dist_to_target = norm([x; y_pos] - target);
    
    if t < params.accel_duration
        % Phase 1: Initial constant acceleration
        ds = params.initial_a;
    else
        % Phase 2: Deceleration using 3-degree Log Polynomial Law
        x_norm = dist_to_target / params.initial_distance_ref;
        x_norm = max(0, min(1, x_norm)); % Clamp to [0, 1]
        a = params.poly_a;
        b = params.poly_b;
        c = params.poly_c;
        n_norm = 1 + a + b + c;
        poly_term = 1 + a*x_norm + b*x_norm^2 + c*x_norm^3;
        
        if poly_term <= 0 || n_norm <= 1
            speed_scale = 0;
        else
            speed_scale = log(poly_term) / log(n_norm);
        end
        
        % The desired speed relative to water is determined by the final
        % speed needed to match the dock's velocity, and the deceleration profile.
        s_desired_auv_rel_water = params.final_speed + (params.cruise_speed - params.final_speed) * speed_scale;
        
        % If very close to the target, force speed to the final docking speed
        if dist_to_target <= params.docking_radius
            s_desired_auv_rel_water = params.final_speed;
        end
        
        % Simple proportional control to achieve the desired speed relative to water
        ds = 0.5 * (s_desired_auv_rel_water - s);
    end
    
    % Equations of Motion - add current velocity to AUV's velocity
    % AUV velocity relative to the water
    vx_auv_rel_water = s * cos(psi); 
    vy_auv_rel_water = s * sin(psi);
    
    % Ground velocity is the sum of AUV velocity relative to water and current velocity
    dx = vx_auv_rel_water + params.v_current(1); % x velocity component
    dy = vy_auv_rel_water + params.v_current(2); % y velocity component
    
    dpsi = rc; % Heading rate of change
    
    % Assemble derivative vector for ODE solver
    dydt = [dx; dy; ds; dpsi; d_integral_heading_error; heading_error];
end

% Non-Linear Guidance (Robust Lookahead Version)
function rc = nonlinear_guidance(x, y_pos, psi, s, params, target)
    start = params.start;
    delta = params.lookahead; % Lookahead distance
    Kp = params.K_nlg_p; % Proportional gain
    
    auv_pos = [x; y_pos];
    path_vec = target - start; % Vector along desired path
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
    
    v_ground_des_vec = carrot_point - [x; y_pos];
    v_ground_des_unit = v_ground_des_vec / norm(v_ground_des_vec);
    
    desired_speed_auv_rel_water = s;
    v_auv_rel_water_des_vec = (desired_speed_auv_rel_water * v_ground_des_unit) - params.v_current;
    
    desired_course = atan2(v_auv_rel_water_des_vec(2), v_auv_rel_water_des_vec(1));
    course_error = wrapToPi(desired_course - psi);
    
    rc = Kp * course_error;
end

% Carrot Guidance
function desired_psi = carrot_guidance(x, y_pos, s, params, target)
    start = params.start;
    L = params.lookahead; % Lookahead distance
    
    P = [x; y_pos]; % Current position
    A = start;
    B = target;
    V = B - A; % Path vector
    
    if dot(V, V) < 1e-10
        carrot = B; % If path is degenerate, aim directly at target
    else
        Q = A + max(0, min(1, dot(P - A, V) / dot(V, V))) * V;
        
        to_target = B - Q;
        d = norm(to_target);
        
        if d > 0
            carrot = Q + min(L, d) * (to_target / d);
        else
            carrot = Q; % If at target, stay there
        end
    end
    
    v_ground_des_vec = carrot - [x; y_pos];
    v_ground_des_unit = v_ground_des_vec / norm(v_ground_des_vec);
    
    v_auv_rel_water_des_vec = (s * v_ground_des_unit) - params.v_current;
    
    desired_psi = atan2(v_auv_rel_water_des_vec(2), v_auv_rel_water_des_vec(1));
end

% Plotting and Helper Functions
function plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_NLG, Y_NLG, ...
                         dock_start_pos, v_dock, start, params_carrot)
    % Extract parameters
    lw = 1.5; % Line width for plots
    
    % Define colors for each guidance method
    nlg_color = [0.8500 0.3250 0.0980]; % Orange
    
    % Extract data from simulation results
    x_LOS = Y_LOS(:,1); y_LOS = Y_LOS(:,2); s_LOS = Y_LOS(:,3); psi_LOS = Y_LOS(:,4);
    x_carrot = Y_carrot(:,1); y_carrot = Y_carrot(:,2); s_carrot = Y_carrot(:,3); psi_carrot = Y_carrot(:,4);
    x_NLG = Y_NLG(:,1); y_NLG = Y_NLG(:,2); s_NLG = Y_NLG(:,3); psi_NLG = Y_NLG(:,4);
    
    % Create categorical array for guidance methods
    labels = categorical({'LOS', 'Carrot', 'NLG'});
    labels = reordercats(labels, {'LOS', 'Carrot', 'NLG'});
    
    % Calculate dock trajectory
    T_sim_max = max([T_LOS(end), T_carrot(end), T_NLG(end)]);
    T_dock = 0:0.1:T_sim_max;
    dock_x = dock_start_pos(1) + v_dock(1) * T_dock;
    dock_y = dock_start_pos(2) + v_dock(2) * T_dock;
    
    % Figure 1: Trajectory Comparison
    figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 800, 600]);
    hold on;
    
    % Plot trajectories for each guidance method
    plot(x_LOS, y_LOS, 'r-', 'LineWidth', lw, 'DisplayName', 'LOS');
    plot(x_carrot, y_carrot, 'g-', 'LineWidth', lw, 'DisplayName', 'Carrot');
    plot(x_NLG, y_NLG, 'Color', nlg_color, 'LineWidth', lw, 'DisplayName', 'NLG');
    
    % Plot dock's path
    plot(dock_x, dock_y, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Dock Path');
    
    % Plot reference points
    plot(start(1), start(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'AUV Start');
    
    % Plot the dock's starting position with a star marker
    plot(dock_start_pos(1), dock_start_pos(2), 'p', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'm', 'DisplayName', 'Dock Start');
    
    % Plot the dock's ending position with a triangle marker
    plot(dock_x(end), dock_y(end), '^', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', 'Dock End');
    
    % Add 2D frustum at the end of the simulation
    radius_large = 1.0; % Bigger radius (m)
    radius_small = 0.5; % Smaller radius (m)
    length_frustum = 3.0; % Length of the frustum (m)
    
    frustum_center_end = [dock_x(end); dock_y(end)];
    frustum_axis_end = v_dock / norm(v_dock);
    normal_unit_end = [-frustum_axis_end(2); frustum_axis_end(1)];
    
    p1_e = frustum_center_end + radius_large * normal_unit_end;
    p2_e = frustum_center_end - radius_large * normal_unit_end;
    p3_e = frustum_center_end + length_frustum * frustum_axis_end - radius_small * normal_unit_end;
    p4_e = frustum_center_end + length_frustum * frustum_axis_end + radius_small * normal_unit_end;
    
    x_frustum_e = [p1_e(1), p2_e(1), p3_e(1), p4_e(1), p1_e(1)];
    y_frustum_e = [p1_e(2), p2_e(2), p3_e(2), p4_e(2), p1_e(2)];
    fill(x_frustum_e, y_frustum_e, [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Docking Frustum (End)');
    
    hold off;
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('AUV Trajectory Comparison with Moving Dock');
    legend('Location', 'best');
    
    % Figure 2: Performance Metrics Comparison
    figure('Name', 'Performance Metrics Comparison', 'Position', [950, 100, 800, 700]);
    
    % Distance to target for each method
    dist_error_LOS = arrayfun(@(t,x,y) norm([x,y] - (dock_start_pos' + v_dock'*t)), T_LOS, x_LOS, y_LOS);
    dist_error_carrot = arrayfun(@(t,x,y) norm([x,y] - (dock_start_pos' + v_dock'*t)), T_carrot, x_carrot, y_carrot);
    dist_error_NLG = arrayfun(@(t,x,y) norm([x,y] - (dock_start_pos' + v_dock'*t)), T_NLG, x_NLG, y_NLG);
    
    % Distance to target over time
    subplot(2,2,1);
    hold on;
    plot(T_LOS, dist_error_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, dist_error_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, dist_error_NLG, 'Color', nlg_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Distance to Moving Dock');
    legend('LOS', 'Carrot', 'NLG', 'Location', 'northeast');
    
    % Heading error comparison
    subplot(2,2,2);
    hold on;
    % Heading error for LOS
    target_x_LOS = dock_start_pos(1) + v_dock(1) * T_LOS;
    target_y_LOS = dock_start_pos(2) + v_dock(2) * T_LOS;
    plot(T_LOS, rad2deg(wrapToPi(atan2(target_y_LOS - y_LOS, target_x_LOS - x_LOS) - psi_LOS)), 'r-', 'LineWidth', lw);
    
    % Heading error for carrot guidance
    desired_psi_carrot = zeros(size(x_carrot));
    for i = 1:length(x_carrot)
        target_t = dock_start_pos + v_dock * T_carrot(i);
        desired_psi_carrot(i) = carrot_guidance(x_carrot(i), y_carrot(i), s_carrot(i), params_carrot, target_t);
    end
    plot(T_carrot, rad2deg(wrapToPi(desired_psi_carrot - psi_carrot)), 'g-', 'LineWidth', lw);
    
    % Heading error for NLG
    target_x_NLG = dock_start_pos(1) + v_dock(1) * T_NLG;
    target_y_NLG = dock_start_pos(2) + v_dock(2) * T_NLG;
    plot(T_NLG, rad2deg(wrapToPi(atan2(target_y_NLG - y_NLG, target_x_NLG - x_NLG) - psi_NLG)), 'Color', nlg_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Heading Error (deg)');
    title('Heading Error Comparison');
    ylim([-190, 190]); % Limit y-axis to reasonable range
    
    % Path following performance (cross-track error)
    subplot(2,2,3);
    hold on;
    plot(T_LOS, calculate_cross_track(x_LOS, y_LOS, start, dock_start_pos), 'r-', 'LineWidth', lw);
    plot(T_carrot, calculate_cross_track(x_carrot, y_carrot, start, dock_start_pos), 'g-', 'LineWidth', lw);
    plot(T_NLG, calculate_cross_track(x_NLG, y_NLG, start, dock_start_pos), 'Color', nlg_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Path Following Performance');
    
    % Control effort (turn rate)
    subplot(2,2,4);
    hold on;
    plot(T_LOS, rad2deg([0; diff(Y_LOS(:,4)) ./ diff(T_LOS)]), 'r-', 'LineWidth', lw);
    plot(T_carrot, rad2deg([0; diff(Y_carrot(:,4)) ./ diff(T_carrot)]), 'g-', 'LineWidth', lw);
    plot(T_NLG, rad2deg([0; diff(Y_NLG(:,4)) ./ diff(T_NLG)]), 'Color', nlg_color, 'LineWidth', lw);
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Turn Rate (deg/s)');
    title('Control Effort Comparison');
    
    sgtitle('AUV Guidance: Core Performance Metrics', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Figure 3: Speed and Efficiency Metrics
    figure('Name', 'Speed and Efficiency Metrics', 'Position', [100, 50, 900, 700]);
    
    times = [T_LOS(end), T_carrot(end), T_NLG(end)];
    path_lengths = [calculate_path_length(x_LOS, y_LOS), ...
                    calculate_path_length(x_carrot, y_carrot), ...
                    calculate_path_length(x_NLG, y_NLG)];
    
    % Speed profiles
    subplot(2,2,[1,2]);
    hold on;
    plot(T_LOS, s_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, s_carrot, 'g-', 'LineWidth', lw);
    plot(T_NLG, s_NLG, 'Color', nlg_color, 'LineWidth', lw);
    grid on; hold off;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('AUV Speed Profile (Relative to Water)');
    legend('LOS', 'Carrot', 'NLG', 'Location', 'best');
    
    % Time to target bar chart
    subplot(2,2,3);
    bar(labels, times);
    grid on;
    title('Time to Dock');
    ylabel('Time (s)');
    
    % Path length bar chart
    subplot(2,2,4);
    bar(labels, path_lengths);
    grid on;
    title('Total Path Length');
    ylabel('Distance (m)');
    yline(norm(dock_start_pos - start), 'r--', 'Min. Distance');
    
    sgtitle('AUV Guidance: Speed and Efficiency', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Figure 4: Deceleration Performance Analysis
    figure('Name', 'Final Approach Performance', 'Position', [950, 50, 600, 500]);
    
    final_speeds = [s_LOS(end), s_carrot(end), s_NLG(end)];
    final_distances = [dist_error_LOS(end), dist_error_carrot(end), dist_error_NLG(end)];
    
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
    xlabel('Final Distance to Dock (m)');
    ylabel('Final Speed (m/s)');
    legend('show', 'Location', 'northeast');
    
    sgtitle('AUV Guidance: Final Approach Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    
end
% Helper function: Wrap angle to [-pi, pi] range
function wrapped = wrapToPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end
% Helper function: Calculate path length from coordinates
function len = calculate_path_length(x_coords, y_coords)
    if numel(x_coords) < 2
        len = 0;
        return;
    end
    len = sum(sqrt(diff(x_coords).^2 + diff(y_coords).^2));
end
% Helper function: Calculate cross-track error
function cross_track = calculate_cross_track(x_pos, y_pos, start, target)
    path_vec = target - start;
    path_length = norm(path_vec);
    
    if path_length < 1e-10
        cross_track = norm([x_pos; y_pos] - start);
        return;
    end
    
    path_unit = path_vec / path_length;
    normal_vec = [-path_unit(2); path_unit(1)];
    
    cross_track = zeros(size(x_pos));
    for i = 1:length(x_pos)
        pos_vec = [x_pos(i); y_pos(i)] - start;
        cross_track(i) = abs(dot(pos_vec, normal_vec));
    end
end