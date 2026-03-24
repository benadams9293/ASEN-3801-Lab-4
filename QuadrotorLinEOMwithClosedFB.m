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
function var_dot = QuadrotorLinEOMwithClosedFB(t, var, g, m, I)
    % 1. Get commanded control forces and moments  (3.2)
    [Fc, Gc] = InnerLoopFeedback(var, m, g);

    delta_Fc = zeros(3,1);
    
    % 2. Feed control moments and forces linear EOM (2.2)
    var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, delta_Fc, Gc);
end