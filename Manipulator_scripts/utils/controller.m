function [derror] = controller(t, error, q_d, dq_d, ddq_d, q,dq, k_p, k_d, ...
                            M_mod, V_mod,tau_c,J,atm_drag, ...
                            M_true,V_true,atm_drag_true, i)


e = error(1:2);
de = error(3:4);

%interpolation

qi_d = q_d(i,:)';
dqi_d = dq_d(i, :)';
ddqi_d = ddq_d(i, :)';

q_real = qi_d - e;
dq_real = dqi_d - de;

J = subs(J,q,q_real);

% modelled dynamics
M_mod = subs(M_mod,q,q_real);
V_mod = subs(subs(V_mod,q,q_real),dq,dq_real);
tauc_mod = subs(tau_c,dq,dq_real);

command = ddqi_d + k_d*de + k_p*e;

tau = M_mod*command + V_mod - tauc_mod - J'*atm_drag;

% true dynamics
M_true = subs(M_true,q,q_real);
V_true = subs(subs(V_true,q,q_real),dq,dq_real);
tauc_true = subs(tau_c,dq,dq_real);
dde = ddqi_d - ((M_true)\(- V_true + tauc_true + J'*atm_drag_true + tau));

derror = [de;double(dde)];


end