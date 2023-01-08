function [V_EE,V_tip, Vs] = virtual_manipulator(Vg,R,L,m_links,M)

% INPUTS:
% - Vg: virtual ground
% - R = {R_q forall q}
% - L = {L_q forall q}
% - m_links
% - M: spacecraft mass
% OUTPUT: Virtual manipulator vector chain
m_tip = 1.0;
N = size(R,1);
m = [M,m_links]; %,m_tip];
M_tot = sum(m);

r_0 = R(1,:)*M/M_tot;
Vs = [Vg r_0'];

for i=2:N
    Sum_r=sum(m(1:i))/M_tot;  
    Sum_l=sum(m(1:i-1))/M_tot;
    r_i = R(i,:)*Sum_r;
    l_i = L(i-1,:)*Sum_l;
    V_i = r_i'+l_i';

    Vs(1:3, end+1) = V_i;
end

V_EE = sum(Vs(1:end-1,:),2);
V_tip = V_EE + Vs(end,:);


end
