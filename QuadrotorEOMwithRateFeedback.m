%{
Contributor(s): Jeremy Li
Course number: ASEN 3801
File name: L3801Lab4_14
Created: 03/10/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: use the functions, Analytically derive the trim state and rotor thrust trim values for a constant velocity
translation at 5 m/s East, while maintaining a yaw of 0 deg. Simulate this trim state for 10
secs and verify it produces the expected trim motion.
• Derive the trim state if a yaw of 90 deg is to be maintained instead while translating 5 m/s
East. Verify this trim state in your simulation as well.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}

% Lab Task 2.5
function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
    % Hardcode quadrotor geometry and constants missing from signature
    d = 0.060; % (m)  Radial distance from CG to propeller 
    km = 0.0024; %(N*m/N)  Control moment coefficient
    
    % 1. Get commanded control forces and moments  (2.3)
    [Fc, Gc] = RotationDerivativeFeedback(var, m, g);
    
    % 2. Convert commands to individual motor thrusts (2.4)
    motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    
    % 3. Feed motor thrusts into the nonlinear EOM (1.3)
    var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end
