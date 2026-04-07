function [Fc, Gc] = nonlineQuadrotorEOMwk3(t, var)
%{
Task: 3.7
Contributor(s): Jacob Legg, Jay Berlin, Jeremy Li, Jeff Wik, Benjamin Adams
Course number: ASEN 3801
File name: nonlinQuadrotorEOMwk3.m
Created: 03/06/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: Calculate nonlinear control forces & moments with k3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: Dynamics and Kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
Inputs: t = Time (s), var = State Vector (12x1) [m, rad, m/s, rad/s],
Constants: g (m/s^2), m (kg) 
%}

    % Quadrotor parameters
    m  = 0.068;
    g  = 9.81;

    % Gains
    % Roll gains
    k1_phi = 3.98e-4;
    k2_phi = 5.63e-4;
    % Pitch gains
    k1_theta = 5.14e-4;
    k2_theta = 7.29e-4;
    % Yaw-rate damping
    k_r = 0.004;
    % Outer Loop
    K3_long = ;  
    K3_lat  = ; 

    % Select which case to run
    mode = 'longitudinal';   % 'longitudinal' or 'lateral'

    % Calc states
    theta = var(5);
    phi   = var(6);
    uE = var(7);
    vE = var(8);
    p = var(10);
    q = var(11);
    r = var(12);
    ur = 0;
    vr = 0;

    if t < 2
        if strcmp(mode, 'longitudinal')
            ur = 0.5;
        elseif strcmp(mode, 'lateral')
            vr = 0.5;
        end
    end

    % -----------------------------
    % Outer-loop angle references
    % -----------------------------
    phi_ref   = 0;
    theta_ref = 0;

    if strcmp(mode, 'lateral')
        phi_ref = K3_lat*(vr - vE); % If direction is wrong, flip sign:
    elseif strcmp(mode, 'longitudinal')
        theta_ref = -K3_long*(ur - uE); % If direction is wrong, flip sign
    end

    % Control force
    Fc = [0;
          0;
          m*g];

    % Control moments
    Lc = -k1_phi*p   - k2_phi*(phi   - phi_ref);
    Mc = -k1_theta*q - k2_theta*(theta - theta_ref);
    Nc = -k_r*r;

    Gc = [Lc; Mc; Nc];
end