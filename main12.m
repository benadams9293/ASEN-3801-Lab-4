clc
clear
close all

const = getConst();
% Initial Conditions
x0 = 0;
y0 = 0;
z0 = 0;
phi0 = 0;
phi1 = deg2rad(5);
theta0 = 0;
psi0 = 0;
u0 = 0;
v0 = 0;
w0 = 0;
p0 = 0;
q0 = 0;
r0 = 0;


% Define the initial state vector
initialState1 = [x0;y0;z0;phi0;theta0;psi0;u0;v0;w0;p0;q0;r0];
initialState2 = [x0;y0;z0;phi1;theta0;psi0;u0;v0;w0;p0;q0;r0];
I = [const.I_x, 0, 0; 0, const.I_y, 0; 0, 0, const.I_z];
force = const.m*const.g/4;
motor_forces = [force;force;force;force]; % Placeholder for motor forces
% Set the simulation time and time step
tFinal = 10; % seconds
dt = 0.01;
tSpan = [0 tFinal];
control_input_array = [motor_forces; 0; 0; 0];

[t1, aircraft_state_array1] = ode45(@(t,var) QuadrotorEOM(t, var, const.g, const.m, I, const.d, const.km, const.nu, const.mu, motor_forces), tSpan, initialState1);
[t2, aircraft_state_array2] = ode45(@(t,var) QuadrotorEOM(t, var, const.g, const.m, I, const.d, const.km, const.nu, const.mu, motor_forces), tSpan, initialState2);

PlotAircraftSim(t1, aircraft_state_array1', control_input_array, [1 2 3 4 5 6], 'b-')
PlotAircraftSim(t2, aircraft_state_array2', control_input_array, [1 2 3 4 5 6], 'r-')


function const = getConst()
    % Compiling constants into a structure
    const.g = 9.81;
    const.m = 0.068;
    const.d = 0.06;
    const.km = 0.0024;
    const.I_x = 5.8*10^-5;
    const.I_y = 7.2*10^-5;
    const.I_z = 1.0*10^-4;
    const.nu = 1*10^-3;
    const.mu = 2*10^-6;
    end
