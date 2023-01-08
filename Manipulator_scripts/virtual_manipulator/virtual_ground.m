function Vg = virtual_ground(c_n,R,L,m_links,M)

% INPUTS:
% - c_n(0): distances between the inertail RF and the  
% - R = {R_q forall q}
% - L = {L_q forall q}
% - m_links
% - M: spacecraft mass
% OUTPUT: Virtual ground

m_tip = 1;

N = size(R,1);
m = [M,m_links,m_tip];
M_tot = sum(m);
Vg = 0;
k = 0; 

Sum = 0;
for j=1:N
    Sum = Sum + (R(j, :) + L(j, :));
end
Vg = (c_n-Sum')*(M/M_tot);

for i=2:N
    Sum = 0;
    for j=i:N
        Sum = Sum + (R(j,:)+L(j,:));
    end

    Vg = Vg + (c_n - Sum')*m(i)/M_tot;
end
end


