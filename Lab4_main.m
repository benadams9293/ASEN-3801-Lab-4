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

%% Task 2.1 
nonlinear_IC_changes = ...
   [0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    deg2rad(5) 0 0 0 0 0; %roll
    0 deg2rad(5) 0 0 0; %pitch
    0 0 deg2rad(5) 0 0 0; %yaw
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 .1 0 0; %roll rate
    0 0 0 0 .1 0; %pitch rate
    0 0 0 0 0 .1]; %yaw rate
nonlinear_IC = zeros(12,1);
for i = 1:6
    x_dot = QuadrotorEOM();
    PlotAircraftSim()
end
