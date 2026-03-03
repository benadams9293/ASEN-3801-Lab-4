%{
Task: 2.2
Contributor(s): Ben Adams
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

%}

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)


end