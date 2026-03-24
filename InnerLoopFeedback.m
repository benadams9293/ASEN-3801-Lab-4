function [Fc, Gc] = InnerLoopFeedback(var, m, g)
%{
Task: 3.2
Contributor(s): Ben Adams, Jacob Legg, Jeremy Li, Jeff Wik, Jay Berlin
Course number: ASEN 3801
File name: InnerLoopFeedback.m
Created: 03/24/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: Create a function to calculate the control vectors Fc and Gc
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
Inputs:
var = 12xn a/c state vector
m = aircraft mass
g = gravitational acceleration

Outputs:
Fc = 3x1 Control Forces (body z-component only)
Gc = 3x1 body Control Moments

%}
phi = var(4);
theta = var(5);
p = var(10);
q = var(11);
r = var(12);

Fc = [0; 0; -m*g];

Gc = [-3.98*10^-4*p-5.63*10^-4*phi;
      -3.98*10^-4*q-5.63*10^-4*theta;
      -0.004*r];
end

