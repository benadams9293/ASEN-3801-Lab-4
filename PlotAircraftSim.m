%{
Task: 1.1
Contributor(s): Jacob Legg, Jeremy Li, Jeff Wik, Jay Berlin
Course number: ASEN 3801
File name: PlotAircraftSim.m
Created: 03/03/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: Plot the results for full simulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: Create 6 sperate figures of the 4 different blocks that
 make up the a/c state vector, the control inputs, and a 3D flight path.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
Inputs:
aircraft_state_array = 12xn a/c state vector
control_input_array = 4xn control input state vector
fig = 
col = 

Outputs:
Plots of...
    -Inertial Position
    -Attitude/Orientation
    -Inertial Velocity
    -Angular Velocity
    -Control Inputs
    -3D Flgiht Path

%}
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

fig = fig(:);

% Plot Inertial Position
figure(fig(1))
subplot(3,1,1)
plot(time, aircraft_state_array(1,:), col); hold on
xlabel('Time (s)'); ylabel('x_E'); title('Inertial Position (x component)');

subplot(3,1,2)
plot(time, aircraft_state_array(2,:), col); hold on
xlabel('Time (s)'); ylabel('y_E'); title('Inertial Position (y component)');

subplot(3,1,3)
plot(time, aircraft_state_array(3,:), col); hold on
xlabel('Time (s)'); ylabel('z_E'); title('Inertial Position (z component)');

% Plot Attitude
figure(fig(2))
subplot(3,1,1)
plot(time, aircraft_state_array(4,:), col); hold on
xlabel('Time (s)'); ylabel('phi'); title('Attitude');

subplot(3,1,2)
plot(time, aircraft_state_array(5,:), col); hold on
xlabel('Time (s)'); ylabel('theta'); title('Attitude');

subplot(3,1,3)
plot(time, aircraft_state_array(6,:), col); hold on
xlabel('Time (s)'); ylabel('psi'); title('Attitude');

% Plot Inertial Velocity
figure(fig(3))
subplot(3,1,1)
plot(time, aircraft_state_array(7,:), col); hold on
xlabel('Time (s)'); ylabel('u^E'); title('Inertial Velocity (x component)');

subplot(3,1,2)
plot(time, aircraft_state_array(8,:), col); hold on
xlabel('Time (s)'); ylabel('v^E'); title('Inertial Velocity (y component)');

subplot(3,1,3)
plot(time, aircraft_state_array(9,:), col); hold on
xlabel('Time (s)'); ylabel('w^E'); title('Inertial Velocity (z component)');

% Plot Angular Velocity
figure(fig(4))
subplot(3,1,1)
plot(time, aircraft_state_array(10,:), col); hold on
xlabel('Time (s)'); ylabel('p'); title('Angular Velocity (x component)');

subplot(3,1,2)
plot(time, aircraft_state_array(11,:), col); hold on
xlabel('Time (s)'); ylabel('q'); title('Angular Velocity (y component)');

subplot(3,1,3)
plot(time, aircraft_state_array(12,:), col); hold on
xlabel('Time (s)'); ylabel('r'); title('Angular Velocity (z component)');

% Plot Control Inputs
figure(fig(5))
subplot(4,1,1)
plot(time, control_input_array(1,:), col); hold on
xlabel('Time (s)'); ylabel('Z_c'); title('Control Force (z direction)');

subplot(4,1,2)
plot(time, control_input_array(2,:), col); hold on
xlabel('Time (s)'); ylabel('L_c'); title('Control Roll Moment');

subplot(4,1,3)
plot(time, control_input_array(3,:), col); hold on
xlabel('Time (s)'); ylabel('M_c'); title('Control Pitching Moment');

subplot(4,1,4)
plot(time, control_input_array(4,:), col); hold on
xlabel('Time (s)'); ylabel('N_c'); title('Control Yaw Moment');

% Plot Flight Path
figure(fig(6))
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col); hold on;
xlabel('x-pos (m)'); ylabel('y-pos (m)'); zlabel('z-pos (m)'); title('3D Aircraft Path'); set(gca,'ZDir','reverse')
    % Start / finish markers
    plot3(aircraft_state_array(1,1),  aircraft_state_array(2,1),  aircraft_state_array(3,1),  'go', 'MarkerFaceColor','g'); hold on;
    plot3(aircraft_state_array(1,end),aircraft_state_array(2,end),aircraft_state_array(3,end),'ro', 'MarkerFaceColor','r'); hold on;
end
