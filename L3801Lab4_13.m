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
var0_hover = [0; 0; 100; 0; 0; 0; 0; 0; 0; 0; 0; 0];     %zeros(12, 1); 

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




function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

fig = fig(:);

% Plot Inertial Position
figure(fig(1))
subplot(3,1,1)
plot(time, aircraft_state_array(1,:), col); hold on
xlabel('Time (s)'); ylabel('x_E'); title('Inertial Position (x component)');

subplot(3,1,2)
plot(time, aircraft_state_array(2,:), col); hold on
xlabel('Time (s)'); ylabel('y_E'); title('Inertial Position (y component)');

subplot(3,1,3)
plot(time, aircraft_state_array(3,:), col); hold on
xlabel('Time (s)'); ylabel('z_E'); title('Inertial Position (z component)');

% Plot Attitude
figure(fig(2))
subplot(3,1,1)
plot(time, aircraft_state_array(4,:), col); hold on
xlabel('Time (s)'); ylabel('phi'); title('Attitude');

subplot(3,1,2)
plot(time, aircraft_state_array(5,:), col); hold on
xlabel('Time (s)'); ylabel('theta'); title('Attitude');

subplot(3,1,3)
plot(time, aircraft_state_array(6,:), col); hold on
xlabel('Time (s)'); ylabel('psi'); title('Attitude');

% Plot Inertial Velocity
figure(fig(3))
subplot(3,1,1)
plot(time, aircraft_state_array(7,:), col); hold on
xlabel('Time (s)'); ylabel('u^E'); title('Inertial Velocity (x component)');

subplot(3,1,2)
plot(time, aircraft_state_array(8,:), col); hold on
xlabel('Time (s)'); ylabel('v^E'); title('Inertial Velocity (y component)');

subplot(3,1,3)
plot(time, aircraft_state_array(9,:), col); hold on
xlabel('Time (s)'); ylabel('w^E'); title('Inertial Velocity (z component)');

% Plot Angular Velocity
figure(fig(4))
subplot(3,1,1)
plot(time, aircraft_state_array(10,:), col); hold on
xlabel('Time (s)'); ylabel('p'); title('Angular Velocity (x component)');

subplot(3,1,2)
plot(time, aircraft_state_array(11,:), col); hold on
xlabel('Time (s)'); ylabel('q'); title('Angular Velocity (y component)');

subplot(3,1,3)
plot(time, aircraft_state_array(12,:), col); hold on
xlabel('Time (s)'); ylabel('r'); title('Angular Velocity (z component)');

% Plot Control Inputs
figure(fig(5))
subplot(4,1,1)
plot(time, control_input_array(1,:), col); hold on
xlabel('Time (s)'); ylabel('Z_c'); title('Control Force (z direction)');

subplot(4,1,2)
plot(time, control_input_array(2,:), col); hold on
xlabel('Time (s)'); ylabel('L_c'); title('Control Roll Moment');

subplot(4,1,3)
plot(time, control_input_array(3,:), col); hold on
xlabel('Time (s)'); ylabel('M_c'); title('Control Pitching Moment');

subplot(4,1,4)
plot(time, control_input_array(4,:), col); hold on
xlabel('Time (s)'); ylabel('N_c'); title('Control Yaw Moment');

% Plot Flight Path
figure(fig(6))
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col); hold on;
xlabel('x-pos (m)'); ylabel('y-pos (m)'); zlabel('z-pos (m)'); title('3D Aircraft Path'); set(gca,'ZDir','reverse')
    % Start / finish markers
    plot3(aircraft_state_array(1,1),  aircraft_state_array(2,1),  aircraft_state_array(3,1),  'go', 'MarkerFaceColor','g'); hold on;
    plot3(aircraft_state_array(1,end),aircraft_state_array(2,end),aircraft_state_array(3,end),'ro', 'MarkerFaceColor','r'); hold on;
end





function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
var_dot = zeros(12, 1);
x = var(1);
y = var(2);
z = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u = var(7);
v = var(8);
w = var(9);
p = var(10);
q = var(11);
r = var(12);
f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);
velE = [u;v;w];
omegaE = [p;q;r];
I_x = I(1,1);
I_y = I(2,2);
I_z = I(3,3);
   
% DCM
cPs = cosd(psi);
sPs = sind(psi);
cTh = cosd(theta);
sTh = sind(theta);
cPh = cosd(phi);
sPh = sind(phi);

rotationEBd = [cTh*cPs, cTh*sPs, -sTh; 
               cPh*sTh*sPh-sPs*cPh, cPh*cPs + sPh*sPs*sTh, sPh*cTh; 
               sPh*sPs + cPh*sTh*cPs, cPh*sTh*sPs-sPh*cPs, cPh*cTh];

p_dot = rotationEBd*velE;
var_dot(1:3) = p_dot;



% Euler Angle Rates
T_matrix = [1, -sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
euler_dot = T_matrix*omegaE;
var_dot(4:6) = euler_dot;


% Compute the forces and torques acting on the quadrotor

F_gravity = m*g*[-sTh; cTh*sPh; cTh*cPh];
F_thrust = [0;0;(-f1-f2-f3-f4)];
F_net = F_gravity + F_thrust;

var_dot(7:9) = F_net / m - cross(omegaE,velE); % Velocity derivatives

% Compute the angular acceleration
L_c = d/sqrt(2)*(-f1-f2+f3+f4);
M_c = d/sqrt(2)*(f1-f2-f3+f4);
N_c = km*(f1-f2+f3-f4);
I_inv = [1/I_x 0 0; 0 1/I_y 0; 0 0 1/I_z];
control_moment = [L_c/I_x; M_c/I_y; N_c/I_z];

angular_acceleration = I_inv * (control_moment - cross(omegaE, I * omegaE)); 
var_dot(10:12) = angular_acceleration; % Angular velocity derivatives


end
