function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
%{
Task: 2.3
Contributor(s): Ben Adams, Jacob Legg, Jeremy Li, Jeff Wik, Jay Berlin
Course number: ASEN 3801
File name: PlotAircraftSim.m
Created: 03/10/2026
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
p = var(10);
q = var(11);
r = var(12);

Fc = [0; 0; m*g];

Gc = [-0.004*p;
      -0.004*q;
      -0.004*r];
end