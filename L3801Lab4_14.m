%{
Contributor(s): Jeremy Li
Course number: ASEN 3801
File name: L3801Lab4_14
Created: 03/10/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: use the functions, Analytically derive the trim state and rotor thrust trim values for a constant velocity
translation at 5 m/s East, while maintaining a yaw of 0 deg. Simulate this trim state for 10
secs and verify it produces the expected trim motion.
• Derive the trim state if a yaw of 90 deg is to be maintained instead while translating 5 m/s
East. Verify this trim state in your simulation as well.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}

% Lab Task 1.4
clear; clc; close all;

% 1. Define Parrot Mambo Parameters 
m = 0.068; g = 9.81; 
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); 
d = 0.060; km = 0.0024; nu = 1e-3; mu = 2e-6;

tspan = [0 10];

%% --- Case 1: 5 m/s East, Yaw = 0 deg ---
% Analytically derived values
phi_trim_1 = asin((nu * 5^2) / (m * g)); 
theta_trim_1 = 0;
psi_trim_1 = 0;

% Body velocities
u_trim_1 = 0;
v_trim_1 = 5 * cos(phi_trim_1);
w_trim_1 = -5 * sin(phi_trim_1);

% Motor forces 
T_1 = m * g * cos(phi_trim_1);
trim_motor_forces_1 = [1; 1; 1; 1] * (T_1 / 4);

% Initial State Vector: [x; y; z; phi; theta; psi; u; v; w; p; q; r]
var0_case1 = [0; 0; -100; phi_trim_1; theta_trim_1; psi_trim_1; u_trim_1; v_trim_1; w_trim_1; 0; 0; 0];

[t1, state1] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, trim_motor_forces_1), tspan, var0_case1);

% Control Inputs for plotting [Zc; Lc; Mc; Nc]
control_input_1 = repmat([-T_1; 0; 0; 0], 1, length(t1));

% Plot 1 in Blue
fig_nums_1 = [1 2 3 4 5 6];
PlotAircraftSim(t1', state1', control_input_1, fig_nums_1, 'b-');


%% --- Case 2: 5 m/s East, Yaw = 90 deg ---
% Analytically derived values
phi_trim_2 = 0; 
theta_trim_2 = asin(-(nu * 5^2) / (m * g)); % Negative pitch to move forward
psi_trim_2 = pi/2; % 90 degrees in radians

% Body velocities
u_trim_2 = 5 * cos(theta_trim_2);
v_trim_2 = 0;
w_trim_2 = 5 * sin(theta_trim_2);

% Motor forces 
T_2 = m * g * cos(theta_trim_2);
trim_motor_forces_2 = [1; 1; 1; 1] * (T_2 / 4);

% Initial State Vector
var0_case2 = [0; 0; -100; phi_trim_2; theta_trim_2; psi_trim_2; u_trim_2; v_trim_2; w_trim_2; 0; 0; 0];

[t2, state2] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, trim_motor_forces_2), tspan, var0_case2);

% Control Inputs for plotting
control_input_2 = repmat([-T_2; 0; 0; 0], 1, length(t2));

% Plot 2 in Red
fig_nums_2 = [7 8 9 10 11 12];
PlotAircraftSim(t2', state2', control_input_2, fig_nums_2, 'r-');