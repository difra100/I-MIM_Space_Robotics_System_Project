function [traj_q,traj_dq,traj_ddq] = quintic_poly_traj(q_i, q_f,  dq_i, dq_f, ddq_i, ddq_f, t,T)

syms a_0 a_1 a_2 a_3 a_4 a_5 real

a = [a_0 a_1 a_2 a_3 a_4 a_5]';
q_t = a_0 + a_1*t + a_2*t^2 + a_3*t^3 + a_4*t^4 + a_5*t^5;
q_t_d = diff(q_t,t);
q_t_dd = diff(q_t_d,t);
system =[];
system = [system; subs(q_t, t, 0)];
system = [system;subs(q_t_d, t, 0)];
system = [system;subs(q_t_dd, t, 0)];
system = [system;subs(q_t, t, T)];
system = [system;subs(q_t_d, t, T)];
system = [system;subs(q_t_dd, t, T)];
[N,~] = size(system);
A = [];
for i=1:N
    line = system(i);
    raw = jacobian(line,a);
    A = [A;raw];
end

B = [q_i, dq_i, ddq_i, q_f, dq_f, ddq_f]';

sol = linsolve(A, B);
traj_q = subs(q_t,a,sol);
traj_dq = subs(q_t_d,a,sol);
traj_ddq = subs(q_t_dd,a,sol);


% tau = t/T;
% 
% traj_q = (1-tau)^3*(q_i + (3*q_i + dq_i*T)*tau + (ddq_i*T^2 + 6*dq_i*T + 12*q_i)*tau^2/2);
% traj_q = traj + tau^3*(q_f + (3*q_f - dq_f*T)*(1-tau) + (ddq_f*T^2 - 6*dq_f*T + 12*q_f)*(1-tau)^2/2);


end

