%{
Contributor(s): Jacob Legg
Course number: ASEN 3801
File name: L3801Lab4_35.m
Created: 03/27/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: Find k3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
%}
%% Section 3.5
clc
clear
close all;
% Constants
I_x = 5.8*10^-5;
I_y = 7.2*10^-5;
k1x = 3.98*10^-4;
k2x = 5.63*10^-4;
k1y = 4.94*10^-4;
k2y = 6.99*10^-4;
k3 = linspace(1*10^-4,1*10^-5,1000);
% Finds Eigenvalues for all values of k3
for i=1:1000
    % Lateral
    A_x = [0,9.81,0;0,0,1;-k3(i)/I_x,-k2x/I_x,-k1x/I_x];
    % Longitudinal
    A_y = [0,-9.81,0;0,0,1;k3(i)/I_y,-k2y/I_y,-k1y/I_y];
    % [~,D] = eig(A_x);
    [~,D] = eig(A_y);
    % Extract eigenvalues for further analysis
    eigenvalue1(i) = D(1,1);
    eigenvalue2(i) = D(2,2);
    eigenvalue3(i) = D(3,3);
end
figure;
hold on;
% plot(eigenvalue1,0)
plot(eigenvalue2, "LineWidth", 2)
plot(eigenvalue3, "LineWidth", 2)
legend('Lambda 2','Lambda 3')
title("Longitudinal Locus Plot")
chosen = length(eigenvalue2) - length(find(abs(eigenvalue2)<=0.8));
print("locusPlotLong", '-dpng')
% Store the eigenvalue corresponding to the chosen index for further analysis
selectedk3 = k3(chosen);
selectedTau = -1/eigenvalue2(chosen);