% Part 2 of the Project 2
clear; close all; clc;

% -------------------------
% System Parameters
% -------------------------
l1 = 1; l2 = 1; l3 = 1;
m1 = 1; m2 = 1; m3 = 1;

% -------------------------
% Compute symbolic partial derivatives of the Mass matrix M(q)
% Used later for computing Ṁ(q, dq)
% -------------------------
dMqFun = generateDMqFun();

% -------------------------
% Control Gain Matrices
% Q1 uses basic damping
% Q2 and Q3 use model-dependent damping
% -------------------------
P1 = 1 * eye(3);         % For Q1 control
P2 = 0.72 * eye(3);      % For Q2 and Q3 controls

% -------------------------
% Simulation Settings
% -------------------------
dt = 0.01;
T = 10;
time = 0:dt:T;
numSims = 150;               % Monte Carlo iterations
tolConvergence = 1e-2;       % Threshold to consider angular velocity as "converged"

% -------------------------
% Convergence Counters
% -------------------------
convergedQ1 = 0;
convergedQ2 = 0;
convergedQ31 = 0;
convergedQ32 = 0;
convergedQ33 = 0;

% -------------------------
% Perturbation Matrices for Q3 robustness testing
% These simulate uncertainty in the system's mass matrix
% -------------------------
mPert1 = 3 * ones(3);
mPert2 = -0.5 * ones(3);
mPert3 = -0.95 * ones(3);

% -------------------------
% Monte Carlo Simulation
% -------------------------
for sim = 1:numSims
    % Initial conditions in radians
    q0 = deg2rad([90 30 0]);
    dq0 = deg2rad([0.0 0.0 10]);

    % Simulate each control strategy using ODE45
    [~, q1] = ode45(@(t, y) dynamics(t, y, P1, 'Q1', dMqFun, mPert1), time, [q0'; dq0']);
    [~, q2] = ode45(@(t, y) dynamics(t, y, P2, 'Q2', dMqFun, mPert1), time, [q0'; dq0']);
    [~, q31] = ode45(@(t, y) dynamics(t, y, P2, 'Q3', dMqFun, mPert1), time, [q0'; dq0']);
    [~, q32] = ode45(@(t, y) dynamics(t, y, P2, 'Q3', dMqFun, mPert2), time, [q0'; dq0']);
    [~, q33] = ode45(@(t, y) dynamics(t, y, P2, 'Q3', dMqFun, mPert3), time, [q0'; dq0']);

    % Check convergence at final time step (angular velocity near 0)
    if norm(q1(end, 4:6)) < tolConvergence
        convergedQ1 = convergedQ1 + 1;
    end
    if norm(q2(end, 4:6)) < tolConvergence
        convergedQ2 = convergedQ2 + 1;
    end
    if norm(q31(end, 4:6)) < tolConvergence
        convergedQ31 = convergedQ31 + 1;
    end
    if norm(q32(end, 4:6)) < tolConvergence
        convergedQ32 = convergedQ32 + 1;
    end
    if norm(q33(end, 4:6)) < tolConvergence
        convergedQ33 = convergedQ33 + 1;
    end
end

% -------------------------
% Convergence Summary Results
% -------------------------
fprintf('Convergence rate Q1: %.2f%%\n', (convergedQ1 / numSims) * 100);
fprintf('Convergence rate Q2: %.2f%%\n', (convergedQ2 / numSims) * 100);
fprintf('Convergence rate Q3 first case: %.2f%%\n', (convergedQ31 / numSims) * 100);
fprintf('Convergence rate Q3 second case: %.2f%%\n', (convergedQ32 / numSims) * 100);
fprintf('Convergence rate Q3 third case: %.2f%%\n', (convergedQ33 / numSims) * 100);

% -------------------------
% Extract Angular Velocities (dq) from each simulation
% -------------------------
dTheta1 = q1(:, 4:6);
dTheta2 = q2(:, 4:6);
dTheta31 = q31(:, 4:6);
dTheta32 = q32(:, 4:6);
dTheta33 = q33(:, 4:6);

% -------------------------
% Compute the mass matrix M(q2) over time (for Q2 and Q3 torque computation)
% -------------------------
massMatrixTime = zeros(3, 3, length(time));
for k = 1:length(time)
    thetaK = q2(k, 1:3).';
    massMatrixTime(:, :, k) = mass_matrix(thetaK);
end

% -------------------------
% Compute the control inputs Q1, Q2, Q3 for plotting
% -------------------------
Q1Vals = zeros(length(time), 3);
Q2Vals = zeros(length(time), 3);
Q3Vals1 = zeros(length(time), 3);
Q3Vals2 = zeros(length(time), 3);
Q3Vals3 = zeros(length(time), 3);

for k = 1:length(time)
    % Q1 uses fixed damping
    Q1Vals(k, :) = (-P1 * dTheta1(k, :)')';

    % Q2 uses M(q)
    Q2Vals(k, :) = (-P2 * massMatrixTime(:, :, k) * dTheta2(k, :)')';

    % Q3 uses M(q) + Delta M (with different perturbations)
    gain1 = P2 * (massMatrixTime(:, :, k) + mPert1);
    gain2 = P2 * (massMatrixTime(:, :, k) + mPert2);
    gain3 = P2 * (massMatrixTime(:, :, k) + mPert3);

    Q3Vals1(k, :) = (-gain1 * dTheta31(k, :)')';
    Q3Vals2(k, :) = (-gain2 * dTheta32(k, :)')';
    Q3Vals3(k, :) = (-gain3 * dTheta33(k, :)')';
end

% -------------------------
% Plot Angular Positions, Velocities, and Control Torques
% -------------------------
plotDqQ2(time, q1, q2, Q1Vals, Q2Vals);
plotDqQ3(time, q31, q2, Q3Vals1, Q2Vals);
plotDqQ3(time, q32, q2, Q3Vals2, Q2Vals);
plotDqQ3(time, q33, q2, Q3Vals3, Q2Vals);

% -------------------------
% Create Animated GIFs of Angular Velocity Comparisons
% -------------------------
gif(time, q1, q2, 'convergenceOf1vs2.gif', 'Q1', 'Q2');
gif(time, q2, q31, 'convergenceOf2vs31.gif', 'Q2', 'Q3 case 1');
gif(time, q2, q32, 'convergenceOf2vs32.gif', 'Q2', 'Q3 case 2');
gif(time, q2, q33, 'convergenceOf2vs33.gif', 'Q2', 'Q3 case 3');

function dMqFun = generateDMqFun()
    % Generates a function handle for the symbolic partial derivatives
    % of the mass matrix M with respect to each generalized coordinate theta1, theta2, theta3.

    % Define symbolic variables for joint angles and parameters
    syms theta1 theta2 theta3 real
    syms l1 l2 l3 m1 m2 m3 real

    % Define the symbolic mass matrix M(θ)
    M = sym(zeros(3,3));
    M(1,1) = (m1+m2+m3)*l1^2;
    M(1,2) = (m2+m3)*l1*l2*cos(theta2 - theta1);
    M(1,3) = m3*l1*l3*cos(theta3 - theta1);
    M(2,1) = M(1,2);
    M(2,2) = (m2+m3)*l2^2;
    M(2,3) = m3*l2*l3*cos(theta3 - theta2);
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
    M(3,3) = m3*l3^2;

    % Compute partial derivatives of M with respect to θ1, θ2, and θ3
    dM_theta1 = diff(M, theta1);
    dM_theta2 = diff(M, theta2);
    dM_theta3 = diff(M, theta3);

    % Stack all derivatives into a 9x3 matrix
    dMq = [reshape(dM_theta1,[],1), reshape(dM_theta2,[],1), reshape(dM_theta3,[],1)];

    % Convert symbolic expression into a MATLAB function handle
    dMqFun = matlabFunction(dMq, 'Vars', {theta1, theta2, theta3, l1, l2, l3, m1, m2, m3});
end

function dy = dynamics(t, y, P, mode, dMqFun, M_pert)
    % Computes the state derivative dy = [dtheta; ddtheta] for the manipulator system.
    % Selects the control law depending on mode (Q1, Q2, or Q3).
    
    % State decomposition
    theta = y(1:3);    % Generalized coordinates
    dtheta = y(4:6);   % Generalized velocities

    % Link and mass parameters
    l1 = 1; l2 = 1; l3 = 1;
    m1 = 1; m2 = 1; m3 = 1;

    % Compute mass matrix M(θ)
    M = zeros(3,3);
    M(1,1) = (m1+m2+m3)*l1^2;
    M(1,2) = (m2+m3)*l1*l2*cos(theta(2) - theta(1));
    M(1,3) = m3*l1*l3*cos(theta(3) - theta(1));
    M(2,1) = M(1,2);
    M(2,2) = (m2+m3)*l2^2;
    M(2,3) = m3*l2*l3*cos(theta(3) - theta(2));
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
    M(3,3) = m3*l3^2;

    % Evaluate partial derivatives of M using symbolic function
    dMq = dMqFun(theta(1), theta(2), theta(3), l1, l2, l3, m1, m2, m3);
    dM_theta1 = reshape(dMq(:,1), 3, 3);
    dM_theta2 = reshape(dMq(:,2), 3, 3);
    dM_theta3 = reshape(dMq(:,3), 3, 3);

    % Compute time derivative of mass matrix (Mdot)
    Mdot = dM_theta1*dtheta(1) + dM_theta2*dtheta(2) + dM_theta3*dtheta(3);

    % Define control torque based on mode
    switch mode
        case 'Q1'
            Q = -P * dtheta;
        case 'Q2'
            Q = -P * M * dtheta;
        case 'Q3'
            Q = -P * (M + M_pert) * dtheta;
        otherwise
            error('Invalid control mode. Choose "Q1", "Q2" or "Q3".');
    end

    % Compute angular acceleration using system dynamics
    ddtheta = M \ (Q - Mdot*dtheta + 0.5*(dtheta.'*Mdot*dtheta));

    % Return state derivative
    dy = [dtheta; ddtheta];
end

function M_m = mass_matrix(theta)
    % Computes the numerical mass matrix M(θ) at a given configuration.
    
    % System constants
    l1 = 1; l2 = 1; l3 = 1;
    m1 = 1; m2 = 1; m3 = 1;

    % Construct mass matrix
    M = zeros(3,3);
    M(1,1) = (m1+m2+m3)*l1^2;
    M(1,2) = (m2+m3)*l1*l2*cos(theta(2) - theta(1));
    M(1,3) = m3*l1*l3*cos(theta(3) - theta(1));
    M(2,1) = M(1,2);
    M(2,2) = (m2+m3)*l2^2;
    M(2,3) = m3*l2*l3*cos(theta(3) - theta(2));
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
    M(3,3) = m3*l3^2;

    % Return computed matrix
    M_m = M;
end

function plotDqQ2(time, q1, q2, Q1, Q2)
% Plots and compares joint positions, velocities, and control torques 
% for two control strategies: Q1 and Q2.

% --- Plot 1: Polar angles (joint positions) ---
figure;
hold on
% Plot joint angles for each link under Q1 (dashed) and Q2 (solid)
plot(time, rad2deg(q1(:,1)), '--', 'LineWidth', 1.5); % Link 1, Q1
plot(time, rad2deg(q2(:,1)), '-', 'LineWidth', 1.5);  % Link 1, Q2
plot(time, rad2deg(q1(:,2)), '--', 'LineWidth', 1.5); % Link 2, Q1
plot(time, rad2deg(q2(:,2)), '-', 'LineWidth', 1.5);  % Link 2, Q2
plot(time, rad2deg(q1(:,3)), '--', 'LineWidth', 1.5); % Link 3, Q1
plot(time, rad2deg(q2(:,3)), '-', 'LineWidth', 1.5);  % Link 3, Q2
hold off
ylim([0 100]);
xlabel('time [s]','Interpreter', 'latex')
ylabel('Polar angles [deg]','Interpreter', 'latex')
title('${q}$ vector components', 'Interpreter', 'latex')
grid on
legend('Link 1 - $Q_1$', 'Link 1 - $Q_2$', 'Link 2 - $Q_1$', ...
       'Link 2 - $Q_2$', 'Link 3 - $Q_1$', 'Link 3 - $Q_2$', ...
       'Location', 'best', 'Interpreter', 'latex')

% --- Plot 2: Angular velocities ---
figure;
hold on
% Plot angular velocities for each link under Q1 and Q2
plot(time, rad2deg(q1(:,4)), '--', 'LineWidth', 1.5); % Link 1, Q1
plot(time, rad2deg(q2(:,4)), '-', 'LineWidth', 1.5);  % Link 1, Q2
plot(time, rad2deg(q1(:,5)), '--', 'LineWidth', 1.5); % Link 2, Q1
plot(time, rad2deg(q2(:,5)), '-', 'LineWidth', 1.5);  % Link 2, Q2
plot(time, rad2deg(q1(:,6)), '--', 'LineWidth', 1.5); % Link 3, Q1
plot(time, rad2deg(q2(:,6)), '-', 'LineWidth', 1.5);  % Link 3, Q2
hold off
xlabel('time [s]','Interpreter', 'latex')
ylabel('Angular Velocities [deg/s]','Interpreter', 'latex')
title('$\dot{q}$ vector components', 'Interpreter', 'latex')
grid on
legend('Link 1 - $Q_1$', 'Link 1 - $Q_2$', 'Link 2 - $Q_1$', ...
       'Link 2 - $Q_2$', 'Link 3 - $Q_1$', 'Link 3 - $Q_2$', ...
       'Location', 'best', 'Interpreter', 'latex')

% --- Plot 3: Control torques (Q vectors) ---
figure;
hold on
% Plot control inputs for each joint under Q1 and Q2
plot(time, Q1(:,1), '--', 'LineWidth', 1.5); % Link 1, Q1
plot(time, Q2(:,1), '-', 'LineWidth', 1.5);  % Link 1, Q2
plot(time, Q1(:,2), '--', 'LineWidth', 1.5); % Link 2, Q1
plot(time, Q2(:,2), '-', 'LineWidth', 1.5);  % Link 2, Q2
plot(time, Q1(:,3), '--', 'LineWidth', 1.5); % Link 3, Q1
plot(time, Q2(:,3), '-', 'LineWidth', 1.5);  % Link 3, Q2
hold off
xlabel('time [s]','Interpreter', 'latex')
ylabel('Control Vector $Q$ [Nm]', 'Interpreter', 'latex')
title('Control vector $Q$ components', 'Interpreter', 'latex')
grid on
legend('Link 1 - $Q_1$', 'Link 1 - $Q_2$', 'Link 2 - $Q_1$', ...
       'Link 2 - $Q_2$', 'Link 3 - $Q_1$', 'Link 3 - $Q_2$', ...
       'Location', 'best', 'Interpreter', 'latex')
end

function plotDqQ3(time, q3, q2, Q3, Q2)
% Plots and compares joint positions, velocities, and control torques 
% for control strategies Q3 and Q2.

% --- Plot 1: Polar angles (joint positions) ---
figure;
hold on
% Plot joint angles for each link under Q3 (dashed) and Q2 (solid)
plot(time, rad2deg(q3(:,1)), '--', 'LineWidth', 1.5); % Link 1, Q3
plot(time, rad2deg(q2(:,1)), '-', 'LineWidth', 1.5);  % Link 1, Q2
plot(time, rad2deg(q3(:,2)), '--', 'LineWidth', 1.5); % Link 2, Q3
plot(time, rad2deg(q2(:,2)), '-', 'LineWidth', 1.5);  % Link 2, Q2
plot(time, rad2deg(q3(:,3)), '--', 'LineWidth', 1.5); % Link 3, Q3
plot(time, rad2deg(q2(:,3)), '-', 'LineWidth', 1.5);  % Link 3, Q2
hold off
ylim ([0 100]);
xlabel('time [s]','Interpreter', 'latex')
ylabel('Polar angles [deg]','Interpreter', 'latex')
title('${q}$ vector components', 'Interpreter', 'latex')
grid on
legend('Link 1 - $Q_3$', 'Link 1 - $Q_2$', 'Link 2 - $Q_3$', ...
       'Link 2 - $Q_2$', 'Link 3 - $Q_3$', 'Link 3 - $Q_2$', ...
       'Location', 'best', 'Interpreter', 'latex')

% --- Plot 2: Angular velocities ---
figure;
hold on
% Plot angular velocities for each link under Q3 and Q2
plot(time, rad2deg(q3(:,4)), '--', 'LineWidth', 1.5); % Link 1, Q3
plot(time, rad2deg(q2(:,4)), '-', 'LineWidth', 1.5);  % Link 1, Q2
plot(time, rad2deg(q3(:,5)), '--', 'LineWidth', 1.5); % Link 2, Q3
plot(time, rad2deg(q2(:,5)), '-', 'LineWidth', 1.5);  % Link 2, Q2
plot(time, rad2deg(q3(:,6)), '--', 'LineWidth', 1.5); % Link 3, Q3
plot(time, rad2deg(q2(:,6)), '-', 'LineWidth', 1.5);  % Link 3, Q2
hold off
xlabel('time [s]','Interpreter', 'latex')
ylabel('Angular Velocities [deg/s]','Interpreter', 'latex')
title('$\dot{q}$ vector components', 'Interpreter', 'latex')
grid on
legend('Link 1 - $Q_3$', 'Link 1 - $Q_2$', 'Link 2 - $Q_3$', ...
       'Link 2 - $Q_2$', 'Link 3 - $Q_3$', 'Link 3 - $Q_2$', ...
       'Location', 'best', 'Interpreter', 'latex')

% --- Plot 3: Control torques (Q vectors) ---
figure;
hold on
% Plot control inputs for each joint under Q3 and Q2
plot(time, Q3(:,1), '--', 'LineWidth', 1.5); % Link 1, Q3
plot(time, Q2(:,1), '-', 'LineWidth', 1.5);  % Link 1, Q2
plot(time, Q3(:,2), '--', 'LineWidth', 1.5); % Link 2, Q3
plot(time, Q2(:,2), '-', 'LineWidth', 1.5);  % Link 2, Q2
plot(time, Q3(:,3), '--', 'LineWidth', 1.5); % Link 3, Q3
plot(time, Q2(:,3), '-', 'LineWidth', 1.5);  % Link 3, Q2
hold off
xlabel('time [s]','Interpreter', 'latex')
ylabel('Control Vector $Q$ [Nm]', 'Interpreter', 'latex')
title('Control vector $Q$ components', 'Interpreter', 'latex')
grid on
legend('Link 1 - $Q_3$', 'Link 1 - $Q_2$', 'Link 2 - $Q_3$', ...
       'Link 2 - $Q_2$', 'Link 3 - $Q_3$', 'Link 3 - $Q_2$', ...
       'Location', 'best', 'Interpreter', 'latex')
end

function gif(time, q1, q2, name, CL1, CL2)
    % Generates a GIF animation comparing angular velocities over time
    % for two control strategies (Q1 vs Q2, or Q2 vs Q3, etc.)

    figure('Position', [100, 100, 700, 500]);
    filename = name;

    for k = 1:10:length(time)  % Every 10 steps to reduce GIF size
        clf;
        % Link 1
        subplot(3,1,1)
        plot(time(1:k), rad2deg(q1(1:k,4)), '--', 'LineWidth', 1.5); hold on;
        plot(time(1:k), rad2deg(q2(1:k,4)), '-', 'LineWidth', 1.5);
        title('Link 1: Angular Velocity'); ylabel('\omega_1 [deg/s]');
        legend(CL1, CL2); grid on; xlim([0 time(end)]); 
        
        % Link 2
        subplot(3,1,2)
        plot(time(1:k), rad2deg(q1(1:k,5)), '--', 'LineWidth', 1.5); hold on;
        plot(time(1:k), rad2deg(q2(1:k,5)), '-', 'LineWidth', 1.5);
        title('Link 2: Angular Velocity'); ylabel('\omega_2 [deg/s]');
        grid on; xlim([0 time(end)]);

        % Link 3
        subplot(3,1,3)
        plot(time(1:k), rad2deg(q1(1:k,6)), '--', 'LineWidth', 1.5); hold on;
        plot(time(1:k), rad2deg(q2(1:k,6)), '-', 'LineWidth', 1.5);
        title('Link 3: Angular Velocity'); xlabel('Time [s]'); ylabel('\omega_3 [deg/s]');
        grid on; xlim([0 time(end)]); 

        sgtitle(sprintf('Angular Velocities over Time (t = %.2f s)', time(k)));

        % Create and save frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if k == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    end
    fprintf('GIF saved as %s\n', filename);
end
