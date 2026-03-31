%{
Contributor(s): Jeremy Li
Course number: ASEN 3801
File name: L3801Lab4_14
Created: 03/10/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: Repeat simulations for the setups in Problems 2.1.d through 2.1.f.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}
% Lab Task 2.5

clear; clc; close all;

% Constants
m = 0.068;                  % Mass (kg)
g = 9.81;                   % Gravity (m/s^2)
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); % Inertia matrix (kg*m^2)
d = 0.060;                  % Radial distance to propeller (m)
km = 0.0024;                % Control moment coefficient (N*m/N)
nu = 1e-3;                  % Aerodynamic force coefficient
mu = 2e-6;                  % Aerodynamic moment coefficient
fig_nums_0 = [1, 2, 3, 4, 5, 6, 7]; 

var0_hover = [0; 0; -100; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 
tspan = [0 10]; % in [s]

% Initial condition deviations
var0_changes = ...
   [0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    deg2rad(5) 0 0 0 0 0; 
    0 deg2rad(5) 0 0 0 0; 
    0 0 deg2rad(5) 0 0 0; 
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 .1 0 0; % 4: roll rate
    0 0 0 0 .1 0; % 5: pitch rate
    0 0 0 0 0 .1];% 6: yaw rate

% --- Relax ode45 tolerances to prevent near-zero crashing ---
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);

% Loop through the rate deviations (columns 4, 5, and 6)
for i = 4:6
    var0 = var0_hover + var0_changes(:,i);
    
    % Calculate trim motor forces for steady hover
    trim_thrust_per_motor = (m * g) / 4; 
    trim_motor_forces = [1; 1; 1; 1] * trim_thrust_per_motor;
    Zc = -m * g;

    % Run ode45 for both, passing the relaxed options to the controlled system
    [t_out_uncon, state_out_uncon] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, trim_motor_forces), tspan, var0);
    [t_out_con, state_out_con] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu), tspan, var0, options);
    
    % Format the output array
    aircraft_state_array_uncon = state_out_uncon';
    aircraft_state_array_con = state_out_con';
    
    % Calculate Control Inputs & Motor Forces for Plotting
    control_input_array_uncon = repmat([Zc; 0; 0; 0], 1, length(t_out_uncon));
    motor_forces_uncon = repmat(trim_motor_forces, 1, length(t_out_uncon));
    
    control_input_array_con = zeros(4, length(t_out_con));
    motor_forces_con = zeros(4, length(t_out_con));
    for j = 1:length(t_out_con)
        [Fc, Gc] = RotationDerivativeFeedback(state_out_con(j,:)', m, g);
        control_input_array_con(:, j) = [Fc(3); Gc(1); Gc(2); Gc(3)];
        motor_forces_con(:, j) = ComputeMotorForces(Fc, Gc, d, km);
    end
    
    % Plotting Standard States (Overlaying both systems)
    fig_nums = fig_nums_0 + 7*(i-4); 
    PlotAircraftSim(t_out_uncon', aircraft_state_array_uncon, control_input_array_uncon, fig_nums(1:6), 'b-');
    PlotAircraftSim(t_out_con', aircraft_state_array_con, control_input_array_con, fig_nums(1:6), 'r--');
    
    % Plotting Motor Forces
    figure(fig_nums(7))
    for motor = 1:4
        subplot(4, 1, motor);
        plot(t_out_uncon, motor_forces_uncon(motor, :), 'b-', 'LineWidth', 1.5); hold on;
        plot(t_out_con, motor_forces_con(motor, :), 'r--', 'LineWidth', 1.5);
        ylabel(sprintf('f_%d (N)', motor)); grid on;
        if motor == 1; legend('Uncontrolled', 'Controlled'); end
        if motor == 4; xlabel('Time (s)'); end
    end
end




