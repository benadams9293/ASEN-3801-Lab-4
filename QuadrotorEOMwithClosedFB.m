%{
Contributor(s): Jeremy Li, Jacob Legg
Course number: ASEN 3801
File name: QuadrotorEOMwithClosedFB
Created: 03/24/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: new equation of motion function with the feedback controller from 2.4 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}

% Lab Task 3.3
function var_dot = QuadrotorEOMwithClosedFB(t, var, g, m, I, nu, mu)
    % Hardcode quadrotor geometry and constants missing from signature
    d = 0.060; % (m)  Radial distance from CG to propeller 
    km = 0.0024; %(N*m/N)  Control moment coefficient
    
    % 1. Get commanded control forces and moments  (2.3)
    [Fc, Gc] = InnerLoopFeedback(var, m, g);
    
    % 2. Convert commands to individual motor thrusts (2.4)
    motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    
    % 3. Feed motor thrusts into the nonlinear EOM (1.3)
    var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end