%{
Contributor(s): Jeremy Li
Course number: ASEN 3801
File name: L3801Lab4_13
Created: 03/03/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: use the functions, Verify adding aero forces does not alter the trim state for steady hover.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}


clear; clc; close all;

% Lab Task 1.3
% 1. Define Parrot Mambo Parameters 
m = 0.068;                  % Mass (kg)
g = 9.81;                   % Gravity (m/s^2)
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); % Inertia matrix (kg*m^2)
d = 0.060;                  % Radial distance to propeller (m)
km = 0.0024;                % Control moment coefficient (N*m/N)
nu = 1e-3;                  % Aerodynamic force coefficient
mu = 2e-6;                  % Aerodynamic moment coefficient

% 2. Define Steady Hover Trim State
% State vector: [x; y; z; phi; theta; psi; u; v; w; p; q; r]
var0_hover = [0; 0; -100; 0; 0; 0; 0; 0; 0; 0; 0; 0];     %zeros(12, 1); 

% Calculate trim motor forces for steady hover
trim_thrust_per_motor = (m * g) / 4; % Total thrust must equal weight
trim_motor_forces = [1; 1; 1; 1] * trim_thrust_per_motor;

% 3. Setup Simulation
tspan = [0 10]; % in [s]

% 4. Run ode45 with the updated EOM (now including drag)
[t_out, state_out] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, trim_motor_forces), tspan, var0_hover);

% Format the output array to match PlotAircraftSim requirements (12 x n)
aircraft_state_array = state_out';

% 5. Calculate Control Inputs for Plotting
% For steady hover, Zc = -mg, and Lc, Mc, Nc are 0. 
Zc = -m * g;
control_input_array = repmat([Zc; 0; 0; 0], 1, length(t_out));

% 6. Plot the Results
fig_nums = [1, 2, 3, 4, 5, 6]; % The 6 figure numbers required
plot_color = 'g-';             % Green line for Task 1.3 hover

% Call the plotting function from Task 1.1
PlotAircraftSim(t_out', aircraft_state_array, control_input_array, fig_nums, plot_color);







