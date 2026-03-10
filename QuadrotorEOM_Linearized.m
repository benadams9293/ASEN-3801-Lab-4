function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
%{
Task: 2.2
Contributor(s): 
Course number: ASEN 3801
File name: QuadrotorEOM_Linearized.m
Created: 03/03/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: linearized model of the full quadrotor dynamics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: Steady hover trim state. 
Repeat Problem 2.1 using the linearized dynamics model, and compare linearized and
non-linearized behaviors on the same plots that you obtained for Problem 2.1 
(ie. turn in one final set of plots for the combined results of Problem 2.1
and 2.2, using the same 10 sec simulation window).

Do not plot responses from multiple deviations on the same plot. For example, don't plot
responses to deviations of +5 deg roll and +5 deg pitch on the same plots, but do plot linearized
and nonlinear responses for +5 deg roll on the same plots
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
Inputs:
var = 12x1 column matrix
deltaFc = deviation from the steady hover trim condition
deltaGc = deviation from the steady hover trim condition

Outputs:
var_dot = 12x1 rate of change for a/c state vector
%}

x_dot = var(7);
y_dot = var(8);
z_dot = var(9);
phi_dot = var(10);
theta_dot = var(11);
psi_dot = var(12);
u_dot = -g*var(5);
v_dot = g*var(4);
w_dot = (1/m)*deltaFc(3);
p_dot = (1/I(1,1))*deltaGc(1);
q_dot = (1/I(2,2))*deltaGc(2);
r_dot = (1/I(3,3))*deltaGc(3);

var_dot = [x_dot;
           y_dot;
           z_dot;
           phi_dot;
           theta_dot;
           psi_dot;
           u_dot;
           v_dot;
           w_dot;
           p_dot;
           q_dot;
           r_dot];
end