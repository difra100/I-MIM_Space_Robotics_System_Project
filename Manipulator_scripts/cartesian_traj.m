function traj = cartesian_traj(p_i,p_f, deg)

% This script allows the formulation of a polynomial trajectory in 
% cartesian space of any degree
% INPUTS:
%   - deg: degree of the polynomial
%   - p_i: initial position
%   - q_f: final position
%   - T: time interval

[q1_g, q2_g] = inverse_kinematics(p_i,l_1,l_2)

