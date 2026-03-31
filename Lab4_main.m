%{
Contributor(s): Ben Adams
Course number: ASEN 3801
File name: Lab4_main.m
Created: 03/03/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: linearized model of the full quadrotor dynamics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}
% Constants
m = 0.068;                  % Mass (kg)
g = 9.81;                   % Gravity (m/s^2)
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); % Inertia matrix (kg*m^2)
d = 0.060;                  % Radial distance to propeller (m)
km = 0.0024;                % Control moment coefficient (N*m/N)
nu = 1e-3;                  % Aerodynamic force coefficient
mu = 2e-6;                  % Aerodynamic moment coefficient
fig_nums_0 = [1, 2, 3, 4, 5, 6]; % The 6 figure numbers required

%% Task 2.1
var0_hover = [0; 0; -100; 0; 0; 0; 0; 0; 0; 0; 0; 0];     %zeros(12, 1); 
tspan = [0 10]; % in [s]

var0_changes = ...
   [0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    deg2rad(5) 0 0 0 0 0; %roll
    0 deg2rad(5) 0 0 0 0; %pitch
    0 0 deg2rad(5) 0 0 0; %yaw
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 .1 0 0; %roll rate
    0 0 0 0 .1 0; %pitch rate
    0 0 0 0 0 .1]; %yaw rate

for i = 1:6
    var0_hover = [0; 0; -100; 0; 0; 0; 0; 0; 0; 0; 0; 0];     %zeros(12, 1); 
    var0 = var0_hover+var0_changes(:,i);

    % Calculate trim motor forces for steady hover
    trim_thrust_per_motor = (m * g) / 4; % Total thrust must equal weight
    trim_motor_forces = [1; 1; 1; 1] * trim_thrust_per_motor;
    
    % 3. Setup Simulation
    tspan = [0 10]; % in [s]

    % Deltas
    deltaFc = zeros(3,1);
    deltaGc = zeros(3,1);
    
    % 4. Run ode45 with the updated EOM (now including drag)
    [t_out_nonlinear, state_out_nonlinear] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, trim_motor_forces), tspan, var0);
    [t_out_linear, state_out_linear] = ode45(@(t,var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), tspan, var0);
    
    % Format the output array to match PlotAircraftSim requirements (12 x n)
    aircraft_state_array_nonlinear = state_out_nonlinear';
    aircraft_state_array_linear = state_out_linear';
    
    
    % 5. Calculate Control Inputs for Plotting
    % For steady hover, Zc = -mg, and Lc, Mc, Nc are 0. 
    Zc = -m * g;
    control_input_array_nonlinear = repmat([Zc; 0; 0; 0], 1, length(t_out_nonlinear));
    control_input_array_linear = repmat([Zc; 0; 0; 0], 1, length(t_out_linear));
    

    % Plotting
    fig_nums = fig_nums_0 + 6*(i-1);

    %PlotAircraftSim(t_out_linear',t_out_nonlinear', aircraft_state_array_linear,aircraft_state_array_nonlinear, control_input_array, fig_nums, plot_color);
    PlotAircraftSim(t_out_linear', aircraft_state_array_linear, control_input_array_linear, fig_nums, 'r-')
    PlotAircraftSim(t_out_nonlinear', aircraft_state_array_nonlinear, control_input_array_nonlinear, fig_nums, 'b-')
end

%% Section 3.5
I_x = 5.8*10^-5;
I_y = 7.2*10^-5;
k1x = 3.98*10^-4;
k2x = 5.63*10^-4;
k1y = 4.94*10^-4;
k2y = 6.99*10^-4;
k3 = linspace(1*10^-4,1*10^-5,1000);
for i=1:1000
    A_x = [0,9.81,0;0,0,1;-k3(i)/I_x,-k2x/I_x,-k1x/I_x];
    A_y = [0,9.81,0;0,0,1;-k3(i)/I_y,-k2y/I_y,-k1y/I_y];
    % [~,D] = eig(A_x);
    [~,D] = eig(A_y);
    % Extract eigenvalues for further analysis
    eigenvalue1(i) = D(1,1);
    eigenvalue2(i) = D(2,2);
    eigenvalue3(i) = D(3,3);
end
figure;
hold on;
% plot(eigenvalue1)
plot(eigenvalue2)
plot(eigenvalue3)
legend('2','3')
chosen = 1000 - length(find(abs(eigenvalue2)<=0.8));
% Store the eigenvalue corresponding to the chosen index for further analysis
selectedk3 = k3(chosen);
selectedTau = -1/eigenvalue2(chosen);