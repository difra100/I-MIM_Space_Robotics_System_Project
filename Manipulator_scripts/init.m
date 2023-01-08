
% All initialization for the fixed base banipulator part

addpath 'data';
addpath 'real_manipulator';
addpath 'virtual_manipulator';
addpath 'utils';

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
density = 2000;     %kq/m^3

% tollerances
tollerance_l = 0.001;
tollerance_rad = 0.0001;
tollerance_thick = 0.000001;

% true values
radius_true = radius + randn(1)*tollerance_rad;
thickness_true = thickness + randn(1)*tollerance_thick;
inner_radius_true = radius_true - thickness_true;

%lengths
l1 = 7;
l2 = 5;
l= [l1,l2];
l_true = l + randn(1,2)*tollerance_l;

% CoMs (assuming the liks have uniform mass distribution)
% d1 = [cos(q1)*l1/2; sin(q1)*l1/2;0];        
% d2 = [cos(q1)*l1+ cos(q2)*l2/2; sin(q1)*l1; -sin(q2)*l2/2];

% d1_true = [cos(q1)*l_true(1)/2; sin(q1)*l_true(1)/2;0];
% d2_true = [cos(q1)*l_true(1)+cos(q2)*l_true(2)/2; sin(q1)*l_true(1)+sin(q2)*l_true(2)/2;0];

d1 = [0; 0; l1/2];
d2 = [(l2/2)*cos(q1)*cos(q2); (l2/2)*sin(q1)*cos(q2);l1+ (l2/2)*sin(q1)]; 
d1_true = [0; 0; l_true(1)/2];
d2_true = [(l_true(2)/2)*cos(q1)*cos(q2); (l_true(2)/2)*sin(q1)*cos(q2);l_true(1)+ (l_true(2)/2)*sin(q1)]; 
d_true= [d1_true,d2_true];
d= [d1,d2];
% masses
m1 = 2*pi*radius*l1*thickness*density;
m2 = 2*pi*radius*l2*thickness*density;
m = [m1,m2];
m1_true = 2*pi*radius_true*l1*thickness_true*density;
m2_true = 2*pi*radius_true*l2*thickness_true*density;
m_true = [m1_true,m2_true];

% Inertias  (OK Inerzie - Tommaso)
I1xx = (m1*(radius^2 + inner_radius^2))/2;
I1yy = (m1*(3*(radius^2 + inner_radius^2)+l1^2))/12;
I1zz = I1yy;
I1 = [I1xx,I1yy,I1zz];
I2xx = (m2*(radius^2 + inner_radius^2))/2;
I2yy = (m2*(3*(radius^2 + inner_radius^2)+l2^2))/12;
I2zz = I2yy;
I2 = [I2xx,I2yy,I2zz];

I1xx_true = (m1_true*(radius_true^2 + inner_radius_true^2))/2;
I1yy_true = (m1_true*(3*(radius_true^2 + inner_radius_true^2)+l_true(1)^2))/12;
I1zz_true = I1yy_true;
I1_true = [I1xx_true,I1yy_true,I1zz_true];
I2xx_true = (m2_true*(radius_true^2 + inner_radius_true^2))/2;
I2yy_true = (m2_true*(3*(radius_true^2 + inner_radius_true^2)+l_true(2)^2))/12;
I2zz_true = I2yy_true;
I2_true = [I2xx_true,I2yy_true,I2zz_true];



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
tau_max=[tau1_max,tau2_max];

%  motor inertia (motor referred)
I_m = eye(2)*2e-4;        

% link viscous friction (motor referred) 
B_m = 8.17e-4;

%tension at max torque
i_tau1_max = 0.133;   
i_tau2_max = 0.309;

% gear reduction
ni_1 = 125;      
ni_2 = 100;
ni = [ni_1,ni_2];

%static frictions
tauC_plus = 1.26e-4;
tauC_minus= -7.09e-4;

% dinamic friction coefficiets
% v_f = 5;     %viscous
% c_f = 3;     %Coulomb

%% PLOT PARAMETERS
orbit_ts = 1;

%% CONTROL/TRAJECTORY PARAMETERS
sampling_rate = 20;     % This would determine the discrettization step

k_p=400;            %proportional gain
k_d=2*sqrt(k_p);    %critical damping


%% SPACECRAFT PARAMETERS

% % starlink dimensions
% volume_starlink = 3.2*1.6*0.2;  %m^3
% mass_starlink = 250;    %kg
% density_starlink = mass_starlink/volume_starlink;  %kg/m^3
% 
% % Cubesat
% density_cubesat = 1330; %kg/m^3
% 
% density_s = (density_starlink + density_cubesat)/2;

% Using the same density:
m_s = 500;  %Kg
% L_s = (m_s/density_s)^(1/3); 
L_s = 1;   %m


%% ATMOSPHERIC DRAG PARAMETERS
drag_coeff = 0.3; % http://www.personal.psu.edu/faculty/c/x/cxc11/papers/steady_Antenna_CC.PDF (turbulent flow) laminar : 1.2
Area_Antenna = 9*pi;
mars_density = 10^-14; % Assumed to be constant
spacecraft_velocity = [0; 1000; 0]; % about 1000 m/s on the y direction. 

%% VIRTUAL MANIPULATOR
syms theta_1 theta_2 theta_3 real % RPY(x, y, z)
syms dtheta_1 dtheta_2 dtheta_3 real 
syms ddtheta_1 ddtheta_2 ddtheta_3 real 

theta_v = [theta_1; theta_2; theta_3];


Q_augm = [theta_v;q];







