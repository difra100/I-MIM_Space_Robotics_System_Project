function [traj_q,traj_dq,traj_ddq] = quintic_poly_traj(q_i, q_f,  dq_i, dq_f, ddq_i, ddq_f, t,T)


syms a_0 a_1 a_2 a_3 a_4 a_5 real

syms s real % --> s = t/T in [0,1] (timing law)


% Path planning using quintic polynomials
a = [a_0 a_1 a_2 a_3 a_4 a_5]';
q_s = a_0 + a_1*s*T + a_2*(s*T)^2 + a_3*(s*T)^3 + a_4*(s*T)^4 + a_5*(s*T)^5;
q_s_d = diff(q_s,s);
q_s_dd = diff(q_s_d,s);
system =[];
system = [system; subs(q_s, s, 0)];
system = [system;subs(q_s_d, s, 0)];
system = [system;subs(q_s_dd, s, 0)];
system = [system;subs(q_s, s, 1)];
system = [system;subs(q_s_d, s, 1)];
system = [system;subs(q_s_dd, s, 1)];
[N,~] = size(system);
A = [];
for i=1:N
    line = system(i);
    raw = jacobian(line,a);
    A = [A;raw];
end

B = [q_i, dq_i, ddq_i, q_f, dq_f, ddq_f]';

sol = linsolve(A, B);
path_q = subs(q_s,a,sol);
path_dq = subs(q_s_d,a,sol);
path_ddq = subs(q_s_dd,a,sol);



end

