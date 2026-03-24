%{
Task: 2.4
Contributor(s): Ben Adams, Jacob Legg
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
% Create the M matrix
    M = [-1 -1 -1 -1;...
        -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2);...
        d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2);...
        km -km km -km];
    Zc = Fc(3,1);
    Lc = Gc(1,1);
    Mc = Gc(2,1);
    Nc = Gc(3,1);
% Simple inversion to calculate the motor forces  
    motor_forces = M\[Zc; Lc; Mc; Nc];

end