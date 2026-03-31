%{
Contributor(s): Jeremy Li
Course number: ASEN 3801
File name: L3801Lab4_14
Created: 03/10/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}



function [Fc, Gc] = VelocityReferenceFeedback(t, var)
    m = 0.068;
    g = 9.81;
    
    % HARDCODE GAINS
    %Ix=lat, Iy=lon
    % Lateral (Roll/Sway) Gains from 3.1 and 3.5
    k1_lat = 3.98e-4; 
    k2_lat = 5.63e-4; 
    k3_lat = 2.306e-5; 
    
    % Longitudinal (Pitch/Surge) Gains from 3.1 and 3.5
    k1_lon = 4.94e-4; 
    k2_lon = 6.99e-4; 
    k3_lon = 2.856e-5; % Check sign
    
    k_yaw = 0.004;
    
    phi = var(4); 
    theta = var(5);
    u = var(7); 
    v = var(8);
    p = var(10); 
    q = var(11); 
    r = var(12);
    
    if t <= 2.0
        u_r = 0.5;
        v_r = 0.5;
    else
        u_r = 0.0;
        v_r = 0.0;
    end
    
    Fc = [0; 0; -m*g]; % Control Forces
    
    Lc = -k1_lat*p - k2_lat*phi + k3_lat*(v_r-v); % Control Moments
    
    Mc = -k1_lon*q - k2_lon*theta - k3_lon*(u_r-u); % Longitudinal Control
    
    Nc = -k_yaw*r; % Yaw Control (from 2.3)
    
    Gc = [Lc; Mc; Nc]; % Combine
end