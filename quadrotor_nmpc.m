%% ========================================================================
% UNDERACTUATED ROBOTICS COURSE PROJECT: NMPC QUADROTOR CONTROL
% 
% Key Features:
% 1. Nonlinear Model Predictive Control for 12-state quadrotor
% 2. Obstacle avoidance using Signed Distance Functions (SDF)
% 3. Small-angle approximation for attitude dynamics
% 4. Real-time 3D visualization with animated drone
%
% DESIGN CHOICES & SIMPLIFICATIONS:
% 1. Euler angle kinematics: φ̇=p, θ̇=q, ψ̇=r (small-angle approx)
% 2. Force equations: a_x = θ·T/m, a_y = -φ·T/m (decoupled approx)
% 3. Separate CasADi/numerical dynamics for solver/simulation efficiency
% 4. Exponential barrier functions for obstacle avoidance
%
% The code demonstrates underactuated control: 4 motor inputs → 6 DOF motion
% through attitude-position coupling.
% ========================================================================

%% NMPC Quadrotor - EQUATIONS VERSION
clear; clc; close all;

% === CASADI SETUP === 
try
    import casadi.*
    fprintf('CasADi imported successfully.\n');
catch
    error('CasADi not found. Please install CasADi and add to path.');
end

%% === Environment Setup ===
gridSize = [25 25 10];
start = [1 1 4]';
goal  = [10 14 5]';

% Obstacles
obstacles = [
    4 4 0 1.2 8; 8 4 0 1.0 8; 12 4 0 1.3 8; 16 4 0 1.1 8;
    2 8 0 1.4 8; 6 8 0 1.1 8; 10 8 0 1.2 8; 14 8 0 1.0 8; 18 8 0 1.3 8;
    4 12 0 1.1 8; 8 12 0 1.4 8; 12 12 0 1.0 8; 16 12 0 1.2 8;
    2 16 0 1.3 8; 6 16 0 1.1 8; 10 16 0 1.4 8; 14 16 0 1.2 8; 18 16 0 1.0 8;
];

%% Quadrotor Parameters
m = 1.2; g = 9.81; l = 0.25;
Ixx = 0.034; Iyy = 0.034; Izz = 0.06;
kf = 8.548e-6; km = 1.016e-2;

%% NMPC Parameters
dt = 0.15; T_final = 40; N = 10;
x_goal = [goal; zeros(9,1)];

% Weights
Q = diag([30, 30, 50, 5, 5, 2, 30, 30, 30, 1, 1, 1]);
R = 1.0 * eye(4);
Q_terminal = 3 * Q;

% Control constraints
u_min = 470 * ones(4,1);
u_max = 530 * ones(4,1);

% State constraints
pitch_roll_max = 0.25;
z_min = 0.5; z_max = 8.0;

% Safety margin for obstacles
safety_margin = 0.25;

%% Initial State
x0 = zeros(12,1); 
x0(1:3) = start;
x0(7:9) = [0.1; 0.1; 0];

%% Visualization
figure(1); hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z'); view(45,30);
title('Quadrotor Control: Goal Seeking with Obstacle Avoidance');
xlim([0 gridSize(1)]); ylim([0 gridSize(2)]); zlim([0 gridSize(3)]);

for i=1:size(obstacles,1)
    [Xc,Yc,Zc] = cylinder(obstacles(i,4),20);
    Zc = Zc*obstacles(i,5) + obstacles(i,3);
    surf(Xc+obstacles(i,1), Yc+obstacles(i,2), Zc, ...
         'FaceColor',[0.8 0.2 0.2],'FaceAlpha',0.7,'EdgeAlpha',0.2);
end

plot3(start(1),start(2),start(3),'bo','MarkerSize',10,'MarkerFaceColor','b','DisplayName','Start');
plot3(goal(1),goal(2),goal(3),'g*','MarkerSize',15,'LineWidth',2,'DisplayName','Goal');
trajectory = plot3(nan,nan,nan,'b-','LineWidth',2,'DisplayName','Trajectory');
current_pos = plot3(x0(1),x0(2),x0(3),'ro','MarkerSize',4,'MarkerFaceColor','r','DisplayName','Quadrotor');

% =========== DRONE VISUALIZATION SETUP ===========
% Drone body (cross-shaped arms)
arm_len = 0.6; 
arms = [-arm_len  arm_len  0 0; 0 0 -arm_len arm_len; 0 0 0 0]; 
hArm1 = plot3([0 0],[0 0],[0 0],'k-','LineWidth',3);
hArm2 = plot3([0 0],[0 0],[0 0],'k-','LineWidth',3);

% Propellers (circles at arm ends)
prop_radius = 0.15;
prop_angle = linspace(0, 2*pi, 20);
prop_x = prop_radius * cos(prop_angle);
prop_y = prop_radius * sin(prop_angle);

% Propeller handles (4 propellers) - static circles
hProp = gobjects(4,1);
prop_colors = {[0.2 0.4 0.8], [0.8 0.2 0.2], [0.2 0.8 0.4], [0.8 0.6 0.2]};
for i = 1:4
    hProp(i) = plot3(nan(1,20), nan(1,20), nan(1,20), 'LineWidth', 1.5, ...
                     'Color', prop_colors{i});
end

% Drone body center (small sphere)
hBody = plot3(nan, nan, nan, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
% ================================================

%legend;

%% Main Loop
X_history = x0;
U_history = [];
comp_times = [];

U_warm = repmat(500*ones(4,1),1,N);
x_current = x0;
goal_reached = false;

fprintf('Starting NMPC with Exact Equations...\n');

%% === NMPC LOOP ===
for t = 0:dt:T_final
    if goal_reached, break; end
    
    tic;
    
    % Solve NMPC
    [U_opt, success] = solve_nmpc_exact(x_current, x_goal, obstacles, N, dt,...
        Q, R, Q_terminal, u_min, u_max, pitch_roll_max, z_min, z_max,...
        m, g, l, Ixx, Iyy, Izz, kf, km, U_warm, safety_margin);
    
    comp_time = toc;
    comp_times = [comp_times; comp_time];
    
    if success
        u_apply = U_opt(:,1);
        U_warm = [U_opt(:,2:end), U_opt(:,end)];
        status = 'Ok';
    else
        u_apply = 500 * ones(4,1);
        status = 'Fail';
    end
    
    % Simulate forward
    x_next = rk4_step(@(x,u) quad_dynamics(x,u,m,g,l,Ixx,Iyy,Izz,kf,km), x_current, u_apply, dt);
    
    % Store
    X_history = [X_history, x_next];
    U_history = [U_history, u_apply];
    x_current = x_next;
    
    % Update visualization
    set(current_pos,'XData',x_current(1),'YData',x_current(2),'ZData',x_current(3));
    
    % =========== DRONE VISUALIZATION ===========
    % Calculate yaw direction based on velocity
    vel = x_current(7:9);
    if norm(vel(1:2)) > 0.01
        yaw_dir = atan2(vel(2), vel(1)); % face movement direction
    else
        yaw_dir = x_current(6); % if almost stationary
    end
    
    % Create rotation matrix (full 3D rotation with roll, pitch, yaw)
    phi = x_current(4);   % roll
    theta = x_current(5); % pitch
    psi = yaw_dir;        % yaw
    
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_vis = Rz * Ry * Rx;
    
    % Rotate arms
    rotated_arms = R_vis * arms;
    
    % Update arms positions
    X_arms = rotated_arms(1,:) + x_current(1);
    Y_arms = rotated_arms(2,:) + x_current(2);
    Z_arms = rotated_arms(3,:) + x_current(3);
    
    set(hArm1, 'XData', [X_arms(1) X_arms(2)], ...
               'YData', [Y_arms(1) Y_arms(2)], ...
               'ZData', [Z_arms(1) Z_arms(2)]);
    set(hArm2, 'XData', [X_arms(3) X_arms(4)], ...
               'YData', [Y_arms(3) Y_arms(4)], ...
               'ZData', [Z_arms(3) Z_arms(4)]);
    
    
    % Propeller positions on arms (local coordinates)
    prop_positions_local = [
        arm_len, 0, 0;    % Front-right (propeller 1)
        0, arm_len, 0;    % Front-left (propeller 2)
        -arm_len, 0, 0;   % Rear-left (propeller 3)
        0, -arm_len, 0;   % Rear-right (propeller 4)
    ]';
    
    for i = 1:4
        % Create propeller circle in local coordinates
        prop_circle_local = [prop_x; prop_y; zeros(size(prop_x))];
        
        % Translate to propeller position
        prop_circle_local = prop_circle_local + prop_positions_local(:,i);
        
        % Rotate with drone orientation
        prop_circle_world = R_vis * prop_circle_local;
        
        % Translate to world position
        X_prop = prop_circle_world(1,:) + x_current(1);
        Y_prop = prop_circle_world(2,:) + x_current(2);
        Z_prop = prop_circle_world(3,:) + x_current(3);
        
        set(hProp(i), 'XData', X_prop, 'YData', Y_prop, 'ZData', Z_prop);
    end
    
    % Update body center
    set(hBody, 'XData', x_current(1), 'YData', x_current(2), 'ZData', x_current(3));
    % =================================================
    
    trajectory.XData = [trajectory.XData, x_current(1)];
    trajectory.YData = [trajectory.YData, x_current(2)];
    trajectory.ZData = [trajectory.ZData, x_current(3)];
    drawnow;
    
    % Check goal
    pos_error = norm(x_current(1:3) - goal);
    if pos_error < 1.0
        fprintf('\n Goal reached at t=%.1f s\n', t);
        goal_reached = true;
    end
    
    % Progress
    if mod(t, 2.0) < dt
        fprintf('%s t=%.1f, pos=[%.1f,%.1f,%.1f], error=%.2f, comp=%.3fs\n',...
                status, t, x_current(1), x_current(2), x_current(3), pos_error, comp_time);
    end
end

%% Calculate obstacle distances
min_distances = [];
for i = 1:size(X_history,2)
    pos = X_history(1:3,i);
    distances = zeros(size(obstacles,1),1);
    
    for j = 1:size(obstacles,1)
        obs = obstacles(j,:);
        % Calculate distance to cylinder obstacle
        dx = pos(1) - obs(1);
        dy = pos(2) - obs(2);
        horizontal_dist = sqrt(dx^2 + dy^2) - obs(4); % Subtract radius
        vertical_dist = abs(pos(3) - (obs(3) + obs(5)/2)) - obs(5)/2;
        distances(j) = max(horizontal_dist, vertical_dist);
    end
    
    min_distances = [min_distances; min(distances)];
end

%% Results Figure with All Plots
figure(2);
time_vec = 0:dt:(length(X_history)-1)*dt;

% Position vs time
subplot(3,3,1); 
plot(time_vec,X_history(1:3,:)'); grid on; title('Position vs Time');
xlabel('Time (s)'); ylabel('Position (m)'); 
legend('x','y','z','Location','best');

% Velocity vs time
subplot(3,3,2); 
plot(time_vec,X_history(7:9,:)'); grid on; title('Velocity vs Time');
xlabel('Time (s)'); ylabel('Velocity (m/s)'); 
legend('v_x','v_y','v_z','Location','best');

% Euler angles vs time
subplot(3,3,3);
euler_angles = X_history(4:6,:); % phi, theta, psi
plot(time_vec, euler_angles'); grid on; title('Euler Angles vs Time');
xlabel('Time (s)'); ylabel('Angle (rad)'); 
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)','Location','best');

% Control inputs vs time
subplot(3,3,4); 
if ~isempty(U_history)
    plot(time_vec(1:end-1),U_history'); grid on; title('Control Inputs vs Time');
    xlabel('Time (s)'); ylabel('Thrust'); 
    legend('u1','u2','u3','u4','Location','best');
end

% Computation time
subplot(3,3,5); 
plot(comp_times, 'b-', 'LineWidth', 1.5); grid on; 
title('Computation Time per Step');
xlabel('Step'); ylabel('Time (s)');
hold on;
plot([1, length(comp_times)], [mean(comp_times), mean(comp_times)], 'r--');
legend('Computation Time', sprintf('Mean: %.3f s', mean(comp_times)), 'Location','best');

% Obstacle distance / collision metric
subplot(3,3,6);
plot(time_vec, min_distances, 'r-', 'LineWidth', 1.5); grid on;
title('Minimum Obstacle Distance');
xlabel('Time (s)'); ylabel('Distance (m)');
hold on;
plot([time_vec(1), time_vec(end)], [safety_margin, safety_margin], 'k--', 'LineWidth', 1.2);
ylim([0 max(min_distances)*1.1]);
legend('Min Distance', sprintf('Safety Margin: %.1f m', safety_margin), 'Location','best');

% Angular rates vs time
subplot(3,3,7);
angular_rates = X_history(10:12,:); % p, q, r
plot(time_vec, angular_rates'); grid on; title('Angular Rates vs Time');
xlabel('Time (s)'); ylabel('Rate (rad/s)'); 
legend('p (roll rate)', 'q (pitch rate)', 'r (yaw rate)','Location','best');

% Position error to goal
subplot(3,3,8);
pos_error = vecnorm(X_history(1:3,:) - goal, 2, 1);
plot(time_vec, pos_error, 'b-', 'LineWidth', 1.5); grid on;
title('Position Error to Goal');
xlabel('Time (s)'); ylabel('Error (m)');
hold on;
plot([time_vec(1), time_vec(end)], [1.0, 1.0], 'r--', 'LineWidth', 1.2);
legend('Position Error', 'Goal Threshold (1 m)','Location','best');

% Control effort (total)
subplot(3,3,9);
if ~isempty(U_history)
    control_effort = sum(U_history.^2, 1);
    plot(time_vec(1:end-1), control_effort, 'm-', 'LineWidth', 1.5); grid on;
    title('Total Control Effort');
    xlabel('Time (s)'); ylabel('Σ u_i^2');
    hold on;
    plot([time_vec(1), time_vec(end-1)], [mean(control_effort), mean(control_effort)], 'k--');
    legend('Control Effort', sprintf('Mean: %.0f', mean(control_effort)), 'Location','best');
end

% Set figure size for better visibility
set(gcf, 'Position', [100 100 1400 900]);

% Print warnings and statistics
fprintf('\n========== SIMULATION STATISTICS ==========\n');
fprintf('Final position: [%.2f, %.2f, %.2f]\n', x_current(1), x_current(2), x_current(3));
fprintf('Goal position: [%.2f, %.2f, %.2f]\n', goal(1), goal(2), goal(3));
fprintf('Final position error: %.3f m\n', pos_error(end));
fprintf('Minimum obstacle distance: %.3f m\n', min(min_distances));
fprintf('Mean computation time: %.3f s\n', mean(comp_times));
fprintf('Maximum computation time: %.3f s\n', max(comp_times));

if any(min_distances < safety_margin)
    fprintf(' WARNING: Drone came within %.3f m of obstacles\n', min(min_distances));
    fprintf('   (below safety margin of %.1f m)\n', safety_margin);
else
    fprintf(' SAFE: All obstacle distances > safety margin (%.1f m)\n', safety_margin);
end

fprintf('==========================================\n');

%% NMPC SOLVER - TRADEOFFS & DESIGN
% 1. Horizon: N=10 steps (1.5s) - balances computation vs. preview
% 2. Obstacle cost: Exponential barrier (soft constraint)
% 3. Warm-start: Previous solution accelerates convergence
% 4. Simplified SDF: Cylinder approximation for computational efficiency

%% ================== NMPC SOLVER - EXACT EQUATIONS ==================
function [U_opt, success] = solve_nmpc_exact(x0, x_goal, obstacles, N, dt,...
    Q, R, Q_terminal, u_min, u_max, angle_max, z_min, z_max,...
    m, g, l, Ixx, Iyy, Izz, kf, km, U_warm, safety_margin)
    
    import casadi.*
    
    % === DECISION VARIABLES ===
    opti = Opti();
    X = opti.variable(12, N+1);
    U = opti.variable(4, N);
    
    % === PARAMETERS ===
    x0_param = opti.parameter(12, 1);
    
    % === CONSTRAINTS ===
    
    % 1. Initial condition
    opti.subject_to(X(:,1) == x0_param);
    
    % 2. Dynamics constraints
    for k = 1:N
        x_next = rk4_casadi(X(:,k), U(:,k), dt, m, g, l, Ixx, Iyy, Izz, kf, km);
        opti.subject_to(X(:,k+1) == x_next);
    end
    
    % 3. Control constraints
    for k = 1:N
        for i = 1:4
            opti.subject_to(U(i,k) >= u_min(i));
            opti.subject_to(U(i,k) <= u_max(i));
        end
    end
    
    % 4. State constraints
    for k = 1:N+1
        opti.subject_to(X(3,k) >= z_min);
        opti.subject_to(X(3,k) <= z_max);
        opti.subject_to(X(4,k) >= -angle_max);
        opti.subject_to(X(4,k) <= angle_max);
        opti.subject_to(X(5,k) >= -angle_max);
        opti.subject_to(X(5,k) <= angle_max);
    end
    
    % === COST FUNCTION ===
    cost = 0;
    
    for k = 1:N
        % State error cost
        state_error = X(:,k) - x_goal;
        cost = cost + state_error' * Q * state_error;
        
        % Control effort cost
        u_ref = 500 * ones(4,1);
        control_error = U(:,k) - u_ref;
        cost = cost + control_error' * R * control_error;
        
        % Control smoothness
        if k > 1
            control_rate = U(:,k) - U(:,k-1);
            cost = cost + 0.1 * (control_rate' * control_rate);
        end
        
        % Obstacle avoidance (SIMPLIFIED - no if_else issues)
        for i = 1:size(obstacles,1)
            obs = obstacles(i,:);
            sdf_val = cylinder_sdf_casadi(X(1:3,k), obs(1:3), obs(4), obs(5));
            
            % Simplified exponential barrier (always active, smaller when far)
            cost = cost + 2000 * exp(-sdf_val);
            
            % Additional penalty based on horizontal distance
            horizontal_dist = sqrt((X(1,k)-obs(1))^2 + (X(2,k)-obs(2))^2);
            cost = cost + 500 * exp(-horizontal_dist);
        end
    end
    
    % Terminal cost
    terminal_error = X(:,N+1) - x_goal;
    cost = cost + terminal_error' * Q_terminal * terminal_error;
    
    % === SOLVE ===
    opti.minimize(cost);
    opti.set_value(x0_param, x0);
    
    % Initial guess
    opti.set_initial(U, U_warm);
    X_guess = zeros(12, N+1);
    for k = 1:N+1
        alpha = (k-1)/N;
        X_guess(:,k) = x0 + alpha * (x_goal - x0);
    end
    opti.set_initial(X, X_guess);
    
    % Solver options
    p_opts = struct('expand', true);
    s_opts = struct('max_iter', 500, 'tol', 1e-4, 'print_level', 0);
    opti.solver('ipopt', p_opts, s_opts);
    
    try
        sol = opti.solve();
        U_opt = sol.value(U);
        success = true;
    catch
        U_opt = repmat(500*ones(4,1),1,N);
        success = false;
    end
end

%% DYNAMICS MODEL (Small-Angle Approximation)
% We use two separate implementations:
% 1. quad_dynamics_casadi: For NMPC solver (CasADi symbolic)
% 2. quad_dynamics: For simulation (MATLAB numeric)
%
% NOTE: We assume φ̇ = p, θ̇ = q, ψ̇ = r (valid for small angles < 0.25 rad)
% This simplification makes the optimization more tractable while
% capturing essential underactuated coupling.

%% ================== CASADI COMPATIBLE FUNCTIONS ==================
function x_next = rk4_casadi(x, u, dt, m, g, l, Ixx, Iyy, Izz, kf, km)
    import casadi.*
    
    k1 = quad_dynamics_casadi(x, u, m, g, l, Ixx, Iyy, Izz, kf, km);
    k2 = quad_dynamics_casadi(x + dt/2 * k1, u, m, g, l, Ixx, Iyy, Izz, kf, km);
    k3 = quad_dynamics_casadi(x + dt/2 * k2, u, m, g, l, Ixx, Iyy, Izz, kf, km);
    k4 = quad_dynamics_casadi(x + dt * k3, u, m, g, l, Ixx, Iyy, Izz, kf, km);
    
    x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end

function dxdt = quad_dynamics_casadi(x, u, m, g, l, Ixx, Iyy, Izz, kf, km)
    import casadi.*
    
    phi = x(4); theta = x(5); psi = x(6);
    p = x(10); q = x(11); r = x(12);
    
    % Thrust and torques
    T = kf * sum(u.^2);
    tau = [l * kf * (u(4)^2 - u(2)^2);
           l * kf * (u(3)^2 - u(1)^2);
           km * (u(1)^2 - u(2)^2 + u(3)^2 - u(4)^2)];
    
    % Small-angle approximation
    pos_dot = x(7:9);
    vel_dot = [(theta * T)/m; 
               (-phi * T)/m; 
               T/m - g];
    attitude_dot = [p; q; r];
    
    % Angular rate dynamics
    omega = [p; q; r];
    inertia = diag([Ixx, Iyy, Izz]);
    omega_dot = inv(inertia) * (tau - cross(omega, inertia * omega));
    
    dxdt = [pos_dot; vel_dot; attitude_dot; omega_dot];
end

function d = cylinder_sdf_casadi(pos, center, radius, height)
    import casadi.*
    cx = center(1); cy = center(2); cz = center(3);
    x = pos(1); y = pos(2); z = pos(3);
    
    dx = x - cx; dy = y - cy;
    horizontal_dist = sqrt(dx^2 + dy^2);
    vertical_dist = abs(z - (cz + height/2));
    
    radial = horizontal_dist - radius;
    vertical = vertical_dist - height/2;
    d = fmax(radial, vertical);
end

%% ================== SIMULATION FUNCTIONS ==================
function dxdt = quad_dynamics(x, u, m, g, l, Ixx, Iyy, Izz, kf, km)
    phi = x(4); theta = x(5); psi = x(6);
    p = x(10); q = x(11); r = x(12);
    
    T = kf * sum(u.^2);
    tau = [l * kf * (u(4)^2 - u(2)^2);
           l * kf * (u(3)^2 - u(1)^2);
           km * (u(1)^2 - u(2)^2 + u(3)^2 - u(4)^2)];
    
    % Small-angle approximation
    pos_dot = x(7:9);
    vel_dot = [(theta * T)/m; 
               (-phi * T)/m; 
               T/m - g];
    attitude_dot = [p; q; r];
    
    % Angular rate dynamics
    omega = [p; q; r];
    inertia = diag([Ixx, Iyy, Izz]);
    omega_dot = inv(inertia) * (tau - cross(omega, inertia * omega));
    
    dxdt = [pos_dot; vel_dot; attitude_dot; omega_dot];
end

function x_next = rk4_step(f, x, u, dt)
    k1 = f(x, u);
    k2 = f(x + 0.5*dt*k1, u);
    k3 = f(x + 0.5*dt*k2, u);
    k4 = f(x + dt*k3, u);
    x_next = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end