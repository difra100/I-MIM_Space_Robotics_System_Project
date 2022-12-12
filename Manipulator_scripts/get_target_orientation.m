function [phi, theta, psi] = get_target_orientation(q,R_I_ee, input)


% R_t_ee = R_t_n * R_n_I * R_I_ee = Rrpy(input) * R_n_I * R_DH
% INPUTS:
%   - q: conf (sym)
%   - R_I_ee: rotation from DHmatrix
%   - input = [alpha_t=15° ,beta_t = boh, gamma_t = roba_inutile]
%   - 
% OUTPUT:
%   - [alpha,beta,gamma] s.t. R_euler([alpha,beta,gamma]) = R_ee_t


R_n_I = [1 0 0;
         0 0 1;
         0 -1 0];

R_t_n = euler_rotation('xyz',input);

R_I_t = R_t_n * R_n_I;

% A = [];
% B = [];
% for row=1:3
%     for col=1:3
%         A = [A;R_I_t(row,col)];
%         B = [B; ]
%     end
% end
R_base_I = [0 1 0;-1 0 0;0 0 1]';

R = R_base_I* R_I_t;

cond = 1;
theta = atan2(-R_I_t(3, 1), sqrt(R(3, 2)^2+R(3, 3)^2))*cond + atan2(-R(3, 1), -sqrt(R(3, 2)^2+R(3, 3)^2))*(1-cond);
psi = atan2(R(3, 2)/cos(theta), R(3, 3)/cos(theta));
phi = atan2(R(2, 1)/cos(theta), R(1, 1)/cos(theta));


% q1_t = atan2(-R_I_t(1,2),R_I_t(2,2));
% q2_t = atan2(-R_I_t(3,1),R_I_t(3,3));
% q_target=[q1_t,q2_t];




%q_target = solve(R_I_ee-R_ee_t == 0,q)


end






