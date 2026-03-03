%{
Task: 2.4
Contributor(s): Ben Adams
Course number: ASEN 3801
File name: ComputeMotorForces.m
Created: 03/03/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: linearized model of the full quadrotor dynamics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
Inputs:
d = d is the distance from aircraft CG to each rotor
km = control moment coefficient

Outputs:
motor_forces = 4x1 column vector

%}
function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

end