function auv_docking_simulation()
    % Create user interface for parameter selection
    create_parameter_ui();
    
    % Wait for user to click the Run Simulation button
    uiwait;
end

function create_parameter_ui()
    % Create figure for parameter selection
    fig = figure('Name', 'AUV Docking Simulation Parameters', ...
                 'Position', [100, 100, 500, 600], ...
                 'MenuBar', 'none', ...
                 'NumberTitle', 'off', ...
                 'Resize', 'off');
    
    % Create UI controls
    uicontrol('Style', 'text', 'Position', [20, 560, 200, 20], ...
              'String', 'Select Deceleration Method:', 'HorizontalAlignment', 'left');
    
    decel_method = uicontrol('Style', 'popupmenu', 'Position', [220, 560, 250, 20], ...
                             'String', {'PID', 'LogPoly with Radius', 'LogPoly without Radius'}, ...
                             'Value', 1);
    
    uicontrol('Style', 'text', 'Position', [20, 520, 200, 20], ...
              'String', 'Initial Heading Angle (deg):', 'HorizontalAlignment', 'left');
    
    heading_angle = uicontrol('Style', 'edit', 'Position', [220, 520, 100, 20], ...
                              'String', '225');
    
    uicontrol('Style', 'text', 'Position', [20, 480, 200, 20], ...
              'String', 'Initial Speed (m/s):', 'HorizontalAlignment', 'left');
    
    initial_speed = uicontrol('Style', 'edit', 'Position', [220, 480, 100, 20], ...
                              'String', '1.0');
    
    uicontrol('Style', 'text', 'Position', [20, 440, 200, 20], ...
              'String', 'Acceleration (m/s²):', 'HorizontalAlignment', 'left');
    
    acceleration = uicontrol('Style', 'edit', 'Position', [220, 440, 100, 20], ...
                             'String', '2.0');
    
    uicontrol('Style', 'text', 'Position', [20, 400, 200, 20], ...
              'String', 'Acceleration Duration (s):', 'HorizontalAlignment', 'left');
    
    accel_duration = uicontrol('Style', 'edit', 'Position', [220, 400, 100, 20], ...
                               'String', '5.0');
    
    % Run button
    uicontrol('Style', 'pushbutton', 'Position', [150, 350, 200, 30], ...
              'String', 'Run Simulation', ...
              'Callback', @run_simulation_callback);
    
    % Store UI handles for callback access
    handles = struct();
    handles.decel_method = decel_method;
    handles.heading_angle = heading_angle;
    handles.initial_speed = initial_speed;
    handles.acceleration = acceleration;
    handles.accel_duration = accel_duration;
    handles.fig = fig;
    
    guidata(fig, handles);
end

function run_simulation_callback(~, ~)
    % Get handles and user inputs
    handles = guidata(gcbf);
    
    decel_methods = {'PID', 'LogPoly with Radius', 'LogPoly without Radius'};
    selected_method = decel_methods{get(handles.decel_method, 'Value')};
    psi0 = deg2rad(str2double(get(handles.heading_angle, 'String')));
    initial_v = str2double(get(handles.initial_speed, 'String'));
    initial_a = str2double(get(handles.acceleration, 'String'));
    accel_duration_val = str2double(get(handles.accel_duration, 'String'));
    
    % Close the parameter UI
    close(handles.fig);
    
    % Run the simulation with selected parameters
    run_simulation(selected_method, psi0, initial_v, initial_a, accel_duration_val);
end

function run_simulation(decel_method, psi0, initial_v, initial_a, accel_duration_val)
    % AUV Initial Conditions
    x0 = -100;     % Initial x position (m)
    y0 = -100;     % Initial y position (m)
    start = [x0; y0]; % Starting position vector

    % Docking Target Point
    xt = 0;        % Target x position (m)
    yt = 0;        % Target y position (m)
    target = [xt; yt]; % Target position vector

    % Calculate final cruise speed after acceleration phase
    cruise_speed = initial_v + initial_a * accel_duration_val;

    % AUV and Controller Parameters
    Kp = 1.5;      % Proportional gain for heading
    Ki = 0.05;     % Integral gain for heading
    Kd = 0.03;     % Derivative gain for heading

    % Guidance Parameters
    N_gain = 4.0;  % Proportional Navigation constant
    docking_radius = 5.0; % Radius for successful docking (m)
    lookahead = 1.0; % Lookahead distance for Carrot Chase and NLG (m)
    K_nlg_p = 1.0;    % Proportional gain for NLG

    % Additional parameters based on deceleration method
    switch decel_method
        case 'PID'
            % PID Deceleration parameters
            Kp_s = 3.0;    % Proportional gain for speed
            Ki_s = 0.1;    % Integral gain for speed
            Kd_s = 0.05;   % Derivative gain for speed
            deceleration_radius = 40.0; % Deceleration start radius (m)
            y0_full = [x0; y0; initial_v; psi0; 0; 0; 0; 0]; % State vector
            model_function = @auv_model_pid_decel;
            
        case 'LogPoly with Radius'
            % Log Polynomial with deceleration radius parameters
            deceleration_radius = 40.0; % Deceleration start radius (m)
            y0_full = [x0; y0; initial_v; psi0; 0; 0]; % State vector
            model_function = @auv_model_log_poly_decel_radius;
            
        case 'LogPoly without Radius'
            % Log Polynomial without deceleration radius parameters
            poly_a = 2.0; % Polynomial coefficient a
            poly_b = 2.0; % Polynomial coefficient b
            poly_c = 1.0; % Polynomial coefficient c
            initial_distance_ref = norm(start - target); % Reference distance
            y0_full = [x0; y0; initial_v; psi0; 0; 0]; % State vector
            model_function = @auv_model_log_poly_decel_noradius;
    end

    % Simulation Time
    tspan = [0 200]; % Simulation time span (s)

    % Common parameters structure
    common_params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
                           'target', target, 'docking_radius', docking_radius, ...
                           'cruise_speed', cruise_speed, ...
                           'start', start, 'lookahead', lookahead, ...
                           'initial_a', initial_a, 'accel_duration', accel_duration_val, ...
                           'N_gain', N_gain);

    % Add method-specific parameters
    switch decel_method
        case 'PID'
            common_params.Kp_s = Kp_s;
            common_params.Ki_s = Ki_s;
            common_params.Kd_s = Kd_s;
            common_params.deceleration_radius = deceleration_radius;
        case 'LogPoly with Radius'
            common_params.deceleration_radius = deceleration_radius;
        case 'LogPoly without Radius'
            common_params.poly_a = poly_a;
            common_params.poly_b = poly_b;
            common_params.poly_c = poly_c;
            common_params.initial_distance_ref = initial_distance_ref;
    end

    % Simulation Setup
    options = odeset('Events', @(t,y) docking_events(t, y, target, docking_radius));

    % Run simulations for each guidance algorithm
    guidance_methods = {'LOS', 'carrot', 'NLG', 'TPN', 'RTPN'};
    results = struct();
    
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        fprintf('Running %s Guidance Simulation...\n', method);
        
        params = common_params;
        params.guidance = method;
        
        if strcmp(method, 'NLG')
            params.K_nlg_p = K_nlg_p;
        end
        
        [T, Y] = ode45(@(t,y) model_function(t, y, params), tspan, y0_full, options);
        
        results.(method).T = T;
        results.(method).Y = Y;
    end

    fprintf('Simulations complete. Plotting results...\n');
    
    % Plot results
    plot_comparison(results, target, start, common_params, decel_method);
end

function dydt = auv_model_pid_decel(t, y, params)
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

function dydt = auv_model_log_poly_decel_radius(t, y, params)
    % Unpack state vector
    x   = y(1); % Current x position
    y_pos = y(2); % Current y position
    s   = y(3); % Current forward speed
    psi = y(4); % Current heading angle
    integral_heading_error = y(5); % Integral of heading error (for PID)
    previous_heading_error = y(6); % Previous heading error (for derivative term)
    
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
    
    % Speed Control - Log Polynomial Law for Deceleration
    dist_to_target = sqrt((x - target(1))^2 + (y_pos - target(2))^2);
    
    if t < params.accel_duration
        % Phase 1: Initial constant acceleration
        ds = params.initial_a;
    elseif dist_to_target > params.deceleration_radius
        % Phase 2: Cruising at constant speed
        ds = 0;
    else
        % Phase 3: Deceleration using Log Polynomial Law
        if dist_to_target <= params.docking_radius
            s_desired = 0; % Stop when within docking radius
        else
            % Log Polynomial Law: v = v0 * ((log(r) - log(rd)) / (log(r0) - log(rd)))^n
            n = 2; % Polynomial exponent
            term = (log(dist_to_target) - log(params.docking_radius)) / ...
                   (log(params.deceleration_radius) - log(params.docking_radius));
            s_desired = params.cruise_speed * max(0, term)^n;
        end
        % Simple proportional control to achieve desired speed
        ds = 0.5 * (s_desired - s);
    end
    
    % Equations of Motion
    dx = s * cos(psi); % x velocity component
    dy = s * sin(psi); % y velocity component
    dpsi = rc; % Heading rate of change
    
    % Assemble derivative vector for ODE solver
    dydt = [dx; dy; ds; dpsi; d_integral_heading_error; heading_error];
end

function dydt = auv_model_log_poly_decel_noradius(t, y, params)
    % Unpack state vector
    x   = y(1); % Current x position
    y_pos = y(2); % Current y position
    s   = y(3); % Current forward speed
    psi = y(4); % Current heading angle
    integral_heading_error = y(5); % Integral of heading error (for PID)
    previous_heading_error = y(6); % Previous heading error (for derivative term)
    
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
    
    % Speed Control - Log Polynomial Law for Deceleration
    dist_to_target = sqrt((x - target(1))^2 + (y_pos - target(2))^2);
    
    if t < params.accel_duration
        % Phase 1: Initial constant acceleration
        ds = params.initial_a;
    else
        % Phase 2: Deceleration using 3-degree Log Polynomial Law
        
        % Normalize distance based on the initial reference distance
        x_norm = dist_to_target / params.initial_distance_ref;
        x_norm = max(0, min(1, x_norm)); % Clamp to [0, 1] for robustness

        % Polynomial coefficients
        a = params.poly_a;
        b = params.poly_b;
        c = params.poly_c;

        % Normalization constant
        n_norm = 1 + a + b + c;

        % Calculate the polynomial term
        poly_term = 1 + a*x_norm + b*x_norm^2 + c*x_norm^3;
        
        % ROBUST IMPLEMENTATION
        % The original formula `N^(log(poly)/log(n))` fails when N=1 and is not
        % suitable for docking as it targets a final speed of 1 m/s, not 0.
        % This adapted logic uses the log-polynomial term as a true speed
        % scaling factor that smoothly transitions from 1 down to 0.
        
        if poly_term <= 0 || n_norm <= 1
            speed_scale = 0; % Safety fallback for log domain errors
        else
            % This term correctly scales from 1 (at initial distance) to 0 (at target)
            speed_scale = log(poly_term) / log(n_norm);
        end
        
        % The desired speed is the cruise speed modulated by the scaling factor.
        s_desired = params.cruise_speed * speed_scale;
        
        % If very close to the target, force speed to zero (this remains as a safety)
        if dist_to_target <= params.docking_radius
            s_desired = 0;
        end

        % Simple proportional control to achieve the desired speed
        ds = 0.5 * (s_desired - s);
    end
    
    % Equations of Motion
    dx = s * cos(psi); % x velocity component
    dy = s * sin(psi); % y velocity component
    dpsi = rc; % Heading rate of change
    
    % Assemble derivative vector for ODE solver
    dydt = [dx; dy; ds; dpsi; d_integral_heading_error; heading_error];
end

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

function [value, isterminal, direction] = docking_events(~, y, target, docking_radius)
    % Calculate distance to target
    dist_to_target = sqrt((y(1) - target(1))^2 + (y(2) - target(2))^2);
    value = dist_to_target - docking_radius; % Event occurs when this becomes zero
    isterminal = 1; % Stop the simulation when event occurs
    direction = -1; % Only trigger when value is decreasing (approaching target)
end

function plot_comparison(results, target, start, params, decel_method)
    % Extract parameters
    docking_radius = params.docking_radius;
    lw = 1.5; % Line width for plots
    
    % Define colors for each guidance method
    colors = struct();
    colors.LOS = 'r';
    colors.carrot = 'g';
    colors.NLG = [0.8500, 0.3250, 0.0980]; % Orange
    colors.TPN = [0.4940, 0.1840, 0.5560]; % Purple
    colors.RTPN = [0.9290, 0.6940, 0.1250]; % Yellow
    
    % Extract data from simulation results
    guidance_methods = fieldnames(results);
    
    % Figure 1: Trajectory Comparison
    figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 800, 600]);
    hold on;
    
    % Plot trajectories for each guidance method
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        Y = results.(method).Y;
        plot(Y(:,1), Y(:,2), 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    
    % Plot reference elements
    plot([start(1), target(1)], [start(2), target(2)], 'k--', 'LineWidth', 1.0, 'DisplayName', 'Desired Path');
    plot(target(1), target(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Target');
    plot(start(1), start(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'Start');
    
    % Plot docking zone
    theta_circle = linspace(0, 2*pi, 100);
    plot(target(1) + docking_radius*cos(theta_circle), target(2) + docking_radius*sin(theta_circle), 'c--', 'LineWidth', 1.0, 'DisplayName', 'Docking Radius');
    
    % Plot deceleration zone if applicable
    if isfield(params, 'deceleration_radius')
        plot(target(1) + params.deceleration_radius*cos(theta_circle), target(2) + params.deceleration_radius*sin(theta_circle), '-.', 'Color', '#999900', 'LineWidth', 1.0, 'DisplayName', 'Decel. Zone');
    end
    
    hold off;
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title(['AUV Trajectory Comparison - ' decel_method ' Deceleration']);
    legend('Location', 'best');
    
    % Figure 2: Performance Metrics Comparison
    figure('Name', 'Performance Metrics Comparison', 'Position', [950, 100, 800, 700]);
    
    % Distance to target for each method
    subplot(2,2,1);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        dist_error = sqrt((Y(:,1) - target(1)).^2 + (Y(:,2) - target(2)).^2);
        plot(T, dist_error, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    
    % Add reference lines for docking radius
    yline(docking_radius, 'c--', 'Docking Radius');
    
    % Add reference line for deceleration radius if applicable
    if isfield(params, 'deceleration_radius')
        yline(params.deceleration_radius, 'y-.', 'Decel. Zone');
    end
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Distance to Target');
    legend('Location', 'northeast');
    
    % Heading error comparison
    subplot(2,2,2);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        
        if strcmp(method, 'carrot')
            % For carrot guidance, need to calculate desired heading at each point
            desired_psi = zeros(size(Y,1), 1);
            for j = 1:size(Y,1)
                desired_psi(j) = carrot_guidance(Y(j,1), Y(j,2), params);
            end
            heading_error = rad2deg(wrapToPi(desired_psi - Y(:,4)));
        else
            % For other methods
            heading_error = rad2deg(wrapToPi(atan2(target(2) - Y(:,2), target(1) - Y(:,1)) - Y(:,4)));
        end
        plot(T, heading_error, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Heading Error (deg)');
    title('Heading Error Comparison');
    ylim([-190, 190]); % Limit y-axis to reasonable range
    
    % Path following performance (cross-track error)
    subplot(2,2,3);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        cross_track = calculate_cross_track(Y(:,1), Y(:,2), start, target);
        plot(T, cross_track, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Path Following Performance');
    
    % Control effort (turn rate)
    subplot(2,2,4);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        turn_rate = rad2deg([0; diff(Y(:,4)) ./ diff(T)]);
        plot(T, turn_rate, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Turn Rate (deg/s)');
    title('Control Effort Comparison');
    
    % Add overall title to the figure
    sgtitle(['AUV Guidance: Core Performance Metrics - ' decel_method ' Deceleration'], 'FontSize', 14, 'FontWeight', 'bold');
    
    % Figure 3: Speed and Efficiency Metrics
    figure('Name', 'Speed and Efficiency Metrics', 'Position', [100, 50, 900, 700]);
    
    % Create categorical array for guidance methods
    labels = categorical(guidance_methods);
    labels = reordercats(labels, guidance_methods);
    
    % Extract simulation end times and path lengths
    times = zeros(1, length(guidance_methods));
    path_lengths = zeros(1, length(guidance_methods));
    
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        times(i) = T(end);
        path_lengths(i) = calculate_path_length(Y(:,1), Y(:,2));
    end
    
    % Speed profiles
    subplot(2,2,[1,2]);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        plot(T, Y(:,3), 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    grid on; hold off;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('AUV Speed Profile');
    legend('Location', 'best');
    
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
    sgtitle(['AUV Guidance: Speed and Efficiency - ' decel_method ' Deceleration'], 'FontSize', 14, 'FontWeight', 'bold');

    % --- NEW PLOTS START HERE ---

    % Figure 4: Deceleration Performance Analysis
    if isfield(params, 'deceleration_radius')
        figure('Name', 'Deceleration Performance Analysis', 'Position', [950, 50, 900, 400]);

        % Pre-calculate final speeds and distances
        final_speeds = zeros(1, length(guidance_methods));
        final_distances = zeros(1, length(guidance_methods));
        decel_start_times = NaN(1, length(guidance_methods));

        for i = 1:length(guidance_methods)
            method = guidance_methods{i};
            T = results.(method).T;
            Y = results.(method).Y;
            
            final_speeds(i) = Y(end, 3); % Final speed
            dist_error = sqrt((Y(:,1) - target(1)).^2 + (Y(:,2) - target(2)).^2);
            final_distances(i) = dist_error(end);

            % Find when deceleration starts
            idx = find(dist_error <= params.deceleration_radius, 1, 'first');
            if ~isempty(idx)
                decel_start_times(i) = T(idx);
            end
        end

        % Final approach performance (speed vs distance)
        subplot(1, 2, 1);
        markers = {'o', 's', 'p', 'd', '^'};
        hold on;
        for i = 1:length(labels)
            method = guidance_methods{i};
            plot(final_distances(i), final_speeds(i), ...
                'Marker', markers{i}, ...
                'MarkerEdgeColor', 'k', ...
                'MarkerFaceColor', colors.(method), ...
                'MarkerSize', 10, ...
                'DisplayName', char(labels(i)));
        end
        hold off;
        grid on;
        title('Final Approach Performance');
        xlabel('Final Distance to Target (m)');
        ylabel('Final Speed (m/s)');
        legend('show', 'Location', 'northeast');
        xline(docking_radius, 'k--', 'Target Radius');

        % Deceleration start time bar chart
        subplot(1, 2, 2);
        b_decel = bar(labels, decel_start_times);
        grid on;
        title('Deceleration Start Time');
        ylabel('Time (s)');

        valid_indices = ~isnan(decel_start_times);
        if any(valid_indices)
            text(b_decel.XEndPoints(valid_indices), b_decel.YEndPoints(valid_indices), ...
                 string(round(b_decel.YData(valid_indices), 1)), ...
                 'HorizontalAlignment','center', 'VerticalAlignment','bottom', 'Color', 'k');
        end
        
        sgtitle(['AUV Guidance: Deceleration Analysis - ' decel_method], 'FontSize', 14, 'FontWeight', 'bold');
    end

    % Figure 5: Relative Velocities
    figure('Name', 'Relative Velocities', 'Position', [100, 100, 1200, 900]);
    
    % Relative velocity magnitude
    subplot(3,1,1);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        vx = Y(:,3) .* cos(Y(:,4));
        vy = Y(:,3) .* sin(Y(:,4));
        v_mag = sqrt(vx.^2 + vy.^2);
        plot(T, v_mag, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    hold off; grid on; xlabel('Time (s)'); ylabel('Velocity Magnitude (m/s)');
    title('Relative Velocity Magnitude'); legend;

    % X component of relative velocity
    subplot(3,1,2);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        vx = Y(:,3) .* cos(Y(:,4));
        plot(T, vx, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    hold off; grid on; xlabel('Time (s)'); ylabel('V_x (m/s)');
    title('X Component of Relative Velocity'); legend;

    % Y component of relative velocity
    subplot(3,1,3);
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        vy = Y(:,3) .* sin(Y(:,4));
        plot(T, vy, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    hold off; grid on; xlabel('Time (s)'); ylabel('V_y (m/s)');
    title('Y Component of Relative Velocity'); legend;

    % Figure 6: Range Rate (rdot) to Target
    figure('Name', 'Range Rate (rdot)', 'Position', [100, 100, 1200, 600]);
    
    calculate_rdot = @(Y, target_pos) ...
        -( ...
          ((target_pos(1) - Y(:,1)) .* (Y(:,3) .* cos(Y(:,4)))) + ...
          ((target_pos(2) - Y(:,2)) .* (Y(:,3) .* sin(Y(:,4)))) ...
        ) ./ sqrt((target_pos(1) - Y(:,1)).^2 + (target_pos(2) - Y(:,2)).^2);
    
    hold on;
    for i = 1:length(guidance_methods)
        method = guidance_methods{i};
        T = results.(method).T;
        Y = results.(method).Y;
        rdot = calculate_rdot(Y, target);
        plot(T, rdot, 'Color', colors.(method), 'LineWidth', lw, 'DisplayName', method);
    end
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Range Rate (m/s)');
    title('Range Rate (Closing Velocity) to Target');
    legend('Location', 'southeast');
    yline(0, 'k--', 'DisplayName', 'Zero Closing Velocity');
end

function wrapped = wrapToPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end

function len = calculate_path_length(x_coords, y_coords)
    if numel(x_coords) < 2
        len = 0;
        return;
    end
    % Sum of distances between consecutive points
    len = sum(sqrt(diff(x_coords).^2 + diff(y_coords).^2));
end

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