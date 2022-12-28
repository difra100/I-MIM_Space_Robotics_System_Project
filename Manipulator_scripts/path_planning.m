function [path_q, path_dq,path_ddq]= path_planning(q_i, q_f,  dq_i, dq_f, ddq_i, ddq_f, s)

% Path planning using quintic polynomial

syms a_0 a_1 a_2 a_3 a_4 a_5 real
a = [a_0 a_1 a_2 a_3 a_4 a_5]';

q_s = a_0 + a_1*s + a_2*s^2 + a_3*s^3 + a_4*s^4 + a_5*s^5;
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