function tau_m = torque_motor(dq,ddq,ni,I_m,M,B_m,tau_c)

% INPUTS:
%       - dq = [dq1,dq2]: actual joint speed
%       - ddq = [ddq1,ddq2]: actual joint acc
%       - ni = [ni_1,ni_2]: gear reductions
%       - I_m: inertia matrix (motor side)
%       - M: inertia matrix in the configuration 
%       (q have been already substituted with tha actiual configuration!)
%       - B_m: viscous friction coefficient (motor side)
% OUTPUT: torques erogated (motor side)

dq_m = ni'.*dq';          % joint angle velocity motor side
ddq_m = ni'.*ddq';        % joint angle acceleration motor side

temp1 = [M(1,1)/ni(1)^2, 0; 
         0, M(2,2)/ni(2)^2]; % diag(M(ii)/ni_i^2)

temp2 = [M(1,2)*ddq(1); M(2,1)*ddq(2)];        % M - M_ii

ni_diag = [1/ni(1),0;0,1/ni(2)];  

tau_m = (I_m + temp1)*ddq_m + ni_diag*temp2;



tau_f = B_m*dq_m + tau_c./ni;     %viscous friction

tau_m = tau_m + tau_f;

end