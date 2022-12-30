function Vg = virtual_ground(c_n,R,L,m_links,M)

% INPUTS:
% - c_n(0): distances between the inertail RF and the  
% - R = {R_q forall q}
% - L = {L_q forall q}
% - m_links
% - M: spacecraft mass
% OUTPUT: Virtual ground

N = 3;
m = [M,m_links];
M_tot = sum(m);
Vg = 0;
for i=1:N
    Sum = 0;
    for j=i:N
        Sum = Sum + (R(j)+L(j+1));
    end
    Vg = Vg + (c_n - Sum)*m(i)/M_tot;
end

end


