% AUV Initial Conditions
x0 = -100;     % Initial x position (m)
y0 = -100;     % Initial y position (m)
psi0 = deg2rad(180); % Initial heading (rad)
start = [x0; y0];

% Docking/Loiter Target Point
xt = 0;        % Target x position (m)
yt = 0;        % Target y position (m)
target = [xt; yt];

% AUV Motion Profile Parameters
initial_v = 1.0;      % Initial forward speed at t=0 (m/s)
initial_a = 2.0;      % Constant acceleration for the initial phase (m/s^2)
accel_duration = 5.0; % Duration of the initial acceleration phase (s)
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
N_gain = 4.0;  % Proportional Navigation constant
K_mpn = 0.8;   % Modified proportional gain
docking_radius = 5.0; % Radius for successful docking (m)
deceleration_radius = 40.0; % Radius at which to start decelerating (m)
lookahead = 15.0; % Lookahead distance for Carrot Chase and NLG (m)
K_nlg_p = 1.0;    % Proportional gain for the robust NLG heading controller
K_course_correction = 0.5; % Gain for PN/MPN large heading error correction

% Parameters for Loiter Following
loiter_radius = 20.0; % Desired radius for loitering (m)
loiter_lambda = 0.4;  % Loiter carrot-chasing parameter (rad)

% Simulation Time
tspan = [0 100]; % Simulation time span (s)

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

fprintf('Running Proportional Navigation (PN) Simulation...\n');
params_PN = common_params;
params_PN.guidance = 'PN';
params_PN.N_gain = N_gain;
params_PN.K_course_correction = K_course_correction;
[T_PN, Y_PN] = ode45(@(t,y) auv_model_deceleration(t, y, params_PN), tspan, y0_full, options);

fprintf('Running Modified Proportional Navigation (MPN) Simulation...\n');
params_MPN = common_params;
params_MPN.guidance = 'MPN';
params_MPN.N_gain = N_gain;
params_MPN.K_mpn = K_mpn;
params_MPN.K_course_correction = K_course_correction;
[T_MPN, Y_MPN] = ode45(@(t,y) auv_model_deceleration(t, y, params_MPN), tspan, y0_full, options);

fprintf('Running Non-Linear Guidance (NLG) Simulation...\n');
params_NLG = common_params;
params_NLG.guidance = 'NLG';
params_NLG.K_nlg_p = K_nlg_p;
[T_NLG, Y_NLG] = ode45(@(t,y) auv_model_deceleration(t, y, params_NLG), tspan, y0_full, options);

fprintf('Running Loiter Following Simulation...\n');
params_loiter = common_params;
params_loiter.guidance = 'loiter';
params_loiter.loiter_radius = loiter_radius;
params_loiter.loiter_lambda = loiter_lambda;
% Loiter does not terminate, so run for the full duration without events
[T_loiter, Y_loiter] = ode45(@(t,y) auv_model_deceleration(t, y, params_loiter), tspan, y0_full);

fprintf('Simulations complete. Plotting results...\n');
% Plot Comparison Results
plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_PN, Y_PN, T_MPN, Y_MPN, T_NLG, Y_NLG, T_loiter, Y_loiter, target, start, loiter_radius, params_carrot, params_loiter, params_PN);


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
    if strcmp(params.guidance, 'PN')
        rc = proportional_navigation_guidance(x, y_pos, s, psi, params);
        heading_error = 0; d_integral_heading_error = 0;
    elseif strcmp(params.guidance, 'MPN')
        rc = modified_proportional_navigation_guidance(x, y_pos, s, psi, params);
        heading_error = 0; d_integral_heading_error = 0;
    elseif strcmp(params.guidance, 'NLG')
        rc = nonlinear_guidance(x, y_pos, psi, params);
        heading_error = 0; d_integral_heading_error = 0;
    else
        % PID controller for other guidance laws (LOS, Carrot, Loiter)
        if strcmp(params.guidance, 'carrot')
            desired_psi = carrot_guidance(x, y_pos, params);
        elseif strcmp(params.guidance, 'loiter')
            desired_psi = loiter_guidance(x, y_pos, params);
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
    elseif dist_to_target > params.deceleration_radius && ~strcmp(params.guidance, 'loiter')
        % Phase 2: Cruising at constant speed (after initial acceleration)
        ds = 0;
        speed_error = 0;
        d_integral_speed_error = 0;
    else
        % Phase 3: Deceleration using PID controller
        % This phase starts when the AUV enters the deceleration_radius or for loitering.
        
        if strcmp(params.guidance, 'loiter')
            % For loitering, maintain cruise speed
            s_desired = params.cruise_speed;
        else
            % For docking, calculate desired speed based on distance to target.
            % The desired speed decreases to 0 as the AUV approaches the docking radius.
            if dist_to_target <= params.docking_radius
                s_desired = 0;
            else
                % Scale speed based on remaining distance within the deceleration zone
                dist_in_zone = params.deceleration_radius - params.docking_radius;
                s_desired = params.cruise_speed * sqrt((dist_to_target - params.docking_radius) / dist_in_zone);
            end
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

% Proportional Navigation Guidance
function rc = proportional_navigation_guidance(x, y_pos, s, psi, params)
    target = params.target;
    N = params.N_gain;
    K_cc = params.K_course_correction;
    R_vec = [target(1) - x; target(2) - y_pos];
    R_sq = R_vec(1)^2 + R_vec(2)^2;
    if R_sq < 1e-6, rc = 0; return; end
    Vr_vec = -s * [cos(psi); sin(psi)];
    los_rate = (R_vec(1) * Vr_vec(2) - R_vec(2) * Vr_vec(1)) / R_sq;
    pn_term = N * los_rate;
    los_angle = atan2(R_vec(2), R_vec(1));
    heading_error = wrapToPi(los_angle - psi);
    course_correction_term = 0;
    if abs(heading_error) > (pi / 2), course_correction_term = K_cc * heading_error; end
    rc = pn_term + course_correction_term;
end

% Modified Proportional Navigation Guidance
function rc = modified_proportional_navigation_guidance(x, y_pos, s, psi, params)
    target = params.target;
    start = params.start;
    N = params.N_gain;
    K_mpn = params.K_mpn;
    K_cc = params.K_course_correction;
    R_vec = [target(1) - x; target(2) - y_pos];
    R = norm(R_vec);
    if R < 1e-3, rc = 0; return; end
    Vr_vec = -s * [cos(psi); sin(psi)];
    los_rate = (R_vec(1) * Vr_vec(2) - R_vec(2) * Vr_vec(1)) / (R^2);
    pn_term = N * los_rate;
    los_angle = atan2(R_vec(2), R_vec(1));
    heading_error = wrapToPi(los_angle - psi);
    course_correction_term = 0;
    if abs(heading_error) > (pi/2), course_correction_term = K_cc * heading_error; end
    path_vec = target - start;
    mpn_term = 0;
    if norm(path_vec) >= 1.0
        pos_vec = [x; y_pos] - start;
        cross_prod_z = path_vec(1)*pos_vec(2) - path_vec(2)*pos_vec(1);
        y_err = cross_prod_z / norm(path_vec);
        if s > 0.1, t_go = R / s; else, t_go = R / 1.0; end
        if R >= 30.0
            mpn_term = K_mpn * (y_err / (t_go^2));
            max_mpn_contribution = 0.25;
            mpn_term = max(min(mpn_term, max_mpn_contribution), -max_mpn_contribution);
        end
    end
    rc = pn_term + mpn_term + course_correction_term;
end

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

% Loiter Guidance
function desired_psi = loiter_guidance(x, y_pos, params)
    O=params.target; r=params.loiter_radius; lambda=params.loiter_lambda;
    theta = atan2(y_pos-O(2),x-O(1));
    carrot = O + r*[cos(theta+lambda); sin(theta+lambda)];
    desired_psi = atan2(carrot(2)-y_pos, carrot(1)-x);
end

% Plotting Function
function plot_comparison(T_LOS, Y_LOS, T_carrot, Y_carrot, T_PN, Y_PN, T_MPN, Y_MPN, T_NLG, Y_NLG, T_loiter, Y_loiter, target, start, loiter_radius, params_carrot, params_loiter, ~)
    docking_radius = params_carrot.docking_radius;
    deceleration_radius = params_carrot.deceleration_radius;
    lw = 1.5; % Define a common LineWidth for visibility
    nlg_color = [0.8500 0.3250 0.0980]; % Define NLG color for consistency
    % Extract Data for all algorithms
    x_LOS = Y_LOS(:,1); y_LOS = Y_LOS(:,2); s_LOS = Y_LOS(:,3); psi_LOS = Y_LOS(:,4);
    x_carrot = Y_carrot(:,1); y_carrot = Y_carrot(:,2); s_carrot = Y_carrot(:,3); psi_carrot = Y_carrot(:,4);
    x_PN = Y_PN(:,1); y_PN = Y_PN(:,2); s_PN = Y_PN(:,3); psi_PN = Y_PN(:,4);
    x_MPN = Y_MPN(:,1); y_MPN = Y_MPN(:,2); s_MPN = Y_MPN(:,3); psi_MPN = Y_MPN(:,4);
    x_NLG = Y_NLG(:,1); y_NLG = Y_NLG(:,2); s_NLG = Y_NLG(:,3); psi_NLG = Y_NLG(:,4);
    x_loiter = Y_loiter(:,1); y_loiter = Y_loiter(:,2); s_loiter = Y_loiter(:,3); psi_loiter = Y_loiter(:,4);
    
    % FIGURE 1: Trajectory Comparison
    figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 800, 600]);
    hold on;
    plot(x_LOS, y_LOS, 'r-', 'LineWidth', lw, 'DisplayName', 'LOS Guidance');
    plot(x_carrot, y_carrot, 'g-', 'LineWidth', lw, 'DisplayName', 'Carrot Chase');
    plot(x_PN, y_PN, 'b-', 'LineWidth', lw, 'DisplayName', 'Prop. Navigation');
    plot(x_MPN, y_MPN, 'c-', 'LineWidth', lw, 'DisplayName', 'Mod. Prop. Navigation');
    plot(x_NLG, y_NLG, 'Color', nlg_color, 'LineWidth', lw, 'DisplayName', 'Non-Linear Guidance (NLG)');
    plot(x_loiter, y_loiter, 'm-', 'LineWidth', lw, 'DisplayName', 'Loiter Following');
    plot([start(1), target(1)], [start(2), target(2)], 'k--', 'LineWidth', 1.0, 'DisplayName', 'Desired Path');
    plot(target(1), target(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Target');
    plot(start(1), start(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'Start');
    theta_circle = linspace(0, 2*pi, 100);
    plot(target(1) + loiter_radius*cos(theta_circle), target(2) + loiter_radius*sin(theta_circle), 'k:', 'LineWidth', lw, 'DisplayName', 'Loiter Circle');
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
    dist_error_PN = sqrt((x_PN - target(1)).^2 + (y_PN - target(2)).^2);
    dist_error_MPN = sqrt((x_MPN - target(1)).^2 + (y_MPN - target(2)).^2);
    dist_error_NLG = sqrt((x_NLG - target(1)).^2 + (y_NLG - target(2)).^2);
    dist_from_center_loiter = sqrt((x_loiter - target(1)).^2 + (y_loiter - target(2)).^2);
    plot(T_LOS, dist_error_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, dist_error_carrot, 'g-', 'LineWidth', lw);
    plot(T_PN, dist_error_PN, 'b-', 'LineWidth', lw);
    plot(T_MPN, dist_error_MPN, 'c-', 'LineWidth', lw);
    plot(T_NLG, dist_error_NLG, 'Color', nlg_color, 'LineWidth', lw);
    plot(T_loiter, dist_from_center_loiter, 'm-', 'LineWidth', lw);
    yline(docking_radius, 'c--', 'Docking Radius');
    yline(deceleration_radius, 'y-.', 'Decel. Zone');
    hold off; grid on; xlabel('Time (s)'); ylabel('Distance (m)');
    title('Distance to Target/Center');
    legend('LOS', 'Carrot', 'PN', 'MPN', 'NLG', 'Loiter', 'Location', 'northeast');

    % Heading Error Comparison
    subplot(2,2,2);
    hold on;
    desired_psi_LOS = atan2(target(2) - y_LOS, target(1) - x_LOS);
    heading_error_LOS = wrapToPi(desired_psi_LOS - psi_LOS);
    desired_psi_carrot = zeros(size(x_carrot));
    for i=1:length(x_carrot), desired_psi_carrot(i) = carrot_guidance(x_carrot(i), y_carrot(i), params_carrot); end
    heading_error_carrot = wrapToPi(desired_psi_carrot - psi_carrot);
    desired_psi_PN = atan2(target(2) - y_PN, target(1) - x_PN);
    heading_error_PN = wrapToPi(desired_psi_PN - psi_PN);
    desired_psi_MPN = atan2(target(2) - y_MPN, target(1) - x_MPN);
    heading_error_MPN = wrapToPi(desired_psi_MPN - psi_MPN);
    desired_psi_NLG = atan2(target(2) - y_NLG, target(1) - x_NLG);
    heading_error_NLG = wrapToPi(desired_psi_NLG - psi_NLG);
    desired_psi_loiter = zeros(size(x_loiter));
    for i=1:length(x_loiter), desired_psi_loiter(i) = loiter_guidance(x_loiter(i), y_loiter(i), params_loiter); end
    heading_error_loiter = wrapToPi(desired_psi_loiter - psi_loiter);
    plot(T_LOS, rad2deg(heading_error_LOS), 'r-', 'LineWidth', lw);
    plot(T_carrot, rad2deg(heading_error_carrot), 'g-', 'LineWidth', lw);
    plot(T_PN, rad2deg(heading_error_PN), 'b-', 'LineWidth', lw);
    plot(T_MPN, rad2deg(heading_error_MPN), 'c-', 'LineWidth', lw);
    plot(T_NLG, rad2deg(heading_error_NLG), 'Color', nlg_color, 'LineWidth', lw);
    plot(T_loiter, rad2deg(heading_error_loiter), 'm-', 'LineWidth', lw);
    hold off; grid on; xlabel('Time (s)'); ylabel('Heading Error (deg)');
    title('Heading Error Comparison'); ylim([-190, 190]);
    
    % Path Following Performance
    subplot(2,2,3);
    hold on;
    cross_track_LOS = calculate_cross_track(x_LOS, y_LOS, start, target);
    cross_track_carrot = calculate_cross_track(x_carrot, y_carrot, start, target);
    cross_track_PN = calculate_cross_track(x_PN, y_PN, start, target);
    cross_track_MPN = calculate_cross_track(x_MPN, y_MPN, start, target);
    cross_track_NLG = calculate_cross_track(x_NLG, y_NLG, start, target);
    cross_track_loiter = abs(dist_from_center_loiter - loiter_radius);
    plot(T_LOS, cross_track_LOS, 'r-', 'LineWidth', lw);
    plot(T_carrot, cross_track_carrot, 'g-', 'LineWidth', lw);
    plot(T_PN, cross_track_PN, 'b-', 'LineWidth', lw);
    plot(T_MPN, cross_track_MPN, 'c-', 'LineWidth', lw);
    plot(T_NLG, cross_track_NLG, 'Color', nlg_color, 'LineWidth', lw);
    plot(T_loiter, cross_track_loiter, 'm-', 'LineWidth', lw);
    hold off; grid on; xlabel('Time (s)'); ylabel('Error (m)');
    title('Path Following Performance');
    
    % Control Effort Comparison
    subplot(2,2,4);
    hold on;
    turn_rate_LOS = [0; diff(Y_LOS(:,4)) ./ diff(T_LOS)];
    turn_rate_carrot = [0; diff(Y_carrot(:,4)) ./ diff(T_carrot)];
    turn_rate_PN = [0; diff(Y_PN(:,4)) ./ diff(T_PN)];
    turn_rate_MPN = [0; diff(Y_MPN(:,4)) ./ diff(T_MPN)];
    turn_rate_NLG = [0; diff(Y_NLG(:,4)) ./ diff(T_NLG)];
    turn_rate_loiter = [0; diff(Y_loiter(:,4)) ./ diff(T_loiter)];
    plot(T_LOS, rad2deg(turn_rate_LOS), 'r-', 'LineWidth', lw);
    plot(T_carrot, rad2deg(turn_rate_carrot), 'g-', 'LineWidth', lw);
    plot(T_PN, rad2deg(turn_rate_PN), 'b-', 'LineWidth', lw);
    plot(T_MPN, rad2deg(turn_rate_MPN), 'c-', 'LineWidth', lw);
    plot(T_NLG, rad2deg(turn_rate_NLG), 'Color', nlg_color, 'LineWidth', lw);
    plot(T_loiter, rad2deg(turn_rate_loiter), 'm-', 'LineWidth', lw);
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
    plot(T_PN, s_PN, 'b-', 'LineWidth', lw);
    plot(T_MPN, s_MPN, 'c-', 'LineWidth', lw);
    plot(T_NLG, s_NLG, 'Color', nlg_color, 'LineWidth', lw);
    plot(T_loiter, s_loiter, 'm-', 'LineWidth', lw);
    grid on; hold off;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('AUV Speed Profile');
    legend('LOS', 'Carrot', 'PN', 'MPN', 'NLG', 'Loiter', 'Location', 'best');

    % Calculate Metrics for bar charts
    labels = categorical({'LOS', 'Carrot', 'PN', 'MPN', 'NLG'});
    labels = reordercats(labels, {'LOS', 'Carrot', 'PN', 'MPN', 'NLG'});
    times = [T_LOS(end), T_carrot(end), T_PN(end), T_MPN(end), T_NLG(end)];
    path_lengths = [
        calculate_path_length(x_LOS, y_LOS), ...
        calculate_path_length(x_carrot, y_carrot), ...
        calculate_path_length(x_PN, y_PN), ...
        calculate_path_length(x_MPN, y_MPN), ...
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
    final_speeds = [s_LOS(end), s_carrot(end), s_PN(end), s_MPN(end), s_NLG(end)];
    % Final Distances at docking radius
    final_distances = [dist_error_LOS(end), dist_error_carrot(end), dist_error_PN(end), dist_error_MPN(end), dist_error_NLG(end)];
    
    % Find time when deceleration starts for each method
    idx_decel_LOS = find(dist_error_LOS <= deceleration_radius, 1, 'first');
    time_decel_LOS = T_LOS(idx_decel_LOS);
    
    idx_decel_carrot = find(dist_error_carrot <= deceleration_radius, 1, 'first');
    time_decel_carrot = T_carrot(idx_decel_carrot);
    
    idx_decel_PN = find(dist_error_PN <= deceleration_radius, 1, 'first');
    time_decel_PN = T_PN(idx_decel_PN);
    
    idx_decel_MPN = find(dist_error_MPN <= deceleration_radius, 1, 'first');
    time_decel_MPN = T_MPN(idx_decel_MPN);
    
    idx_decel_NLG = find(dist_error_NLG <= deceleration_radius, 1, 'first');
    time_decel_NLG = T_NLG(idx_decel_NLG);
    
    decel_start_times = [time_decel_LOS, time_decel_carrot, time_decel_PN, time_decel_MPN, time_decel_NLG];

    % Final Approach Performance (Speed vs. Distance)
    subplot(1, 2, 1);
    markers = {'o', 's', 'd', '^', 'p'};
    colors = {'r', 'g', 'b', 'c', nlg_color};
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
