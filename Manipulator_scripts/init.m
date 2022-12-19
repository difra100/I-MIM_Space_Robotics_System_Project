clear all
% All initialization for the fixed base banipulator part

%% SYMBOLIC VARIABLES
% initialize symbolic joint positions/velocities
syms t q1 q2  dq1 dq2 ddq1 ddq2 real  
q = [q1,q2]';
dq = [dq1, dq2]';
ddq = [ddq1,ddq2]';

%% LINKS parameters

radius = 0.1;
thickness = 0.001;
inner_radius = radius - thickness;
density = 2;

%lengths
l1 = 7;
l2 = 5;
l= [l1,l2];

% CoMs (assuming the liks have uniform mass distribution)
d1 = [cos(q1)*l1/2; sin(q1)*l1/2;0];
d2 = [cos(q1)*l1+ cos(q2)*l2/2; sin(q1)*l1+ sin(q2)*l2/2;0];
d= [d1,d2];

% masses
m1 = 2*radius*pi*l1*thickness*density;
m2 = 2*radius*pi*l2*thickness*density;
m = [m1,m2];

% Inertias  (TO CHECK)
I1xx = (m1*(radius^2 + inner_radius^2))/2;
I1yy = (m1*(3*(radius^2 + inner_radius^2)+l1^2))/12;
I1zz = I1yy;
I1 = [I1xx,I1yy,I1zz];

I2xx = (m2*(radius^2 + inner_radius^2))/2;
I2yy = (m2*(3*(radius^2 + inner_radius^2)+l2^2))/12;
I2zz = I2yy;
I2 = [I2xx,I2yy,I2zz];



%% JOINTS parameters
theta1_min = -135 * pi/180;
theta2_min = -138 * pi/180;
theta1_max = 150 * pi/180;
theta2_max= 80 * pi/180;
theta_bounds.high = [theta1_max,theta2_max];
theta_bounds.low = [theta1_min,theta2_min];


%% ACTUATORS parameters

%max torques
tau1_max = 47;
tau2_max = 148;

%inertias
I_m = eye(2)*2e-4;          
B_m = eye(2)*8.17e-4;

%tension at max torque
i_tau1_max = 0.133;   
i_tau2_max = 0.309;

% gear reduction
ni_1 = 125;      
ni_2 = 100;

tauC_max = 1.26e-4;
tauC_min = -7.09e-4;

% friction coefficiets
v_f = 5;     %viscous
c_f = 3;     %Coulomb

%% DYNAMIC PARAMETERS





% Inertia matrices
%I1 = eye(3);
%I2 = eye(3);














