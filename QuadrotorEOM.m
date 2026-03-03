function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
var_dot = zeros(12, 1);
x = var(1);
y = var(2);
z = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u = var(7);
v = var(8);
w = var(9);
p = var(10);
q = var(11);
r = var(12);
f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);
velE = [u;v;w];
omegaE = [p;q;r];
I_x = I(1,1);
I_y = I(2,2);
I_z = I(3,3);
   
% DCM
cPs = cosd(psi);
sPs = sind(psi);
cTh = cosd(theta);
sTh = sind(theta);
cPh = cosd(phi);
sPh = sind(phi);

rotationEBd = [cTh*cPs, cTh*sPs, -sTh; 
               cPh*sTh*sPh-sPs*cPh, cPh*cPs + sPh*sPs*sTh, sPh*cTh; 
               sPh*sPs + cPh*sTh*cPs, cPh*sTh*sPs-sPh*cPs, cPh*cTh];

p_dot = rotationEBd*velE;
var_dot(1:3) = p_dot;



% Euler Angle Rates
T_matrix = [1, -sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
euler_dot = T_matrix*omegaE;
var_dot(4:6) = euler_dot;


% Compute the forces and torques acting on the quadrotor

F_gravity = m*g*[-sTh; cTh*sPh; cTh*cPh];
F_c = [0;0;(-f1-f2-f3-f4)];
F_net = F_gravity + F_c;

var_dot(7:9) = F_net / m - cross(omegaE,velE); % Velocity derivatives

% Compute the angular acceleration
L_c = d/sqrt(2)*(-f1-f2+f3+f4);
M_c = d/sqrt(2)*(f1-f2-f3+f4);
N_c = km*(f1-f2+f3-f4);
I_inv = [1/I_x 0 0; 0 1/I_y 0; 0 0 1/I_z];
control_moment = [L_c/I_x; M_c/I_y; N_c/I_z];

angular_acceleration = I_inv * (control_moment - cross(omegaE, I * omegaE)); 
var_dot(10:12) = angular_acceleration; % Angular velocity derivatives


end