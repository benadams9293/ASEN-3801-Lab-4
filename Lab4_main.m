%{
Contributor(s): 
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
nonlinear_IC = ...
   [0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    deg2rad(5) 0 0 0 0 0; %roll
    0 deg2rad(5) 0 0 0; %pitch
    0 0 0 0 deg2rad(5) 0; %yaw
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
x_dot = QuadrotorEOM();
