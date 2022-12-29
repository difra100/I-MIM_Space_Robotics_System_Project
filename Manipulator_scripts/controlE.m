function [error] = controlE(q,dq,t, e, de, q_d, dq_d, ddq_d, timestep, k_p, k_d, ...
                            M_mod, V_mod,tauc_mod,J,atm_drag, ...
                            M_true,V_true,tauc_true,atm_drag_true)

%interpolation
qi_d = interp1(timestep, q_d, t);
dqi_d = interp1(timestep, dq_d, t);
ddqi_d = interp1(timestep, ddq_d, t);

q_real = qi_d- e;
dq_real = dqi_d - de;

% modelled dynamics
M_mod = subs(M_mod,q,q_real);
V_mod = subs(subs(V_mod,q,q_real),dq,dq_real);

command = ddq1_d + k_d*de + k_p*e;

tau = M_mod*command + V_mod - tauc_mod - J'*atm_drag;


% true dynamics
M_true = subs(M_true,q,q_true);
V_true = subs(subs(V_true,q,q_real),dq,dq_real);

dde = ddqi_d - ((M_true)\(- V_true + tauc_true + J'*atm_drag_true + tau));

error = [de;dde]';

end