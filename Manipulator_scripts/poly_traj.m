function traj = poly_traj(q, q_i, q_f, t, T,deg)
% This script allows the formulation of a polynomial trajectory in 
% configuration space of any degree
% INPUTS:
%   - deg: degree of the polynomial
%   - q_i = [q1_i,q2_i,...]: initial conf
%   - q_f = [q1_f,q2_f,...]: final conf
%   - T: time interval
[~,n_row] = size(q');
a = sym('a',[n_row,deg+1]); assume(a,"real");


%q_t = zeros(n_row,1)
values = [[];[]];
for col=1:deg+1
        values = [values,a(:,col)*t^(col-1)];
        %q_t(row,:) = q_t(row,:) + t^(col-1)* a(row,col)
end

qt=[];
for row=1:n_row
    qt= [qt;[sum(values(row,:))]];
end

n_diff = (deg+1)/2 - 1;
Q_d = [];
for row=1:n_row
    Q_d = [Q_d;qt(row)];
    surplus = (n_diff+1)*(row-1);
    for i=1:n_diff
        Q_d = [Q_d;diff(Q_d(i+surplus,:),t)];
        %Q_d = [Q_d;diff(Q_d(i*2-1:i*2),t)];
    end
end
disp(Q_d)

A_i = [];
A_f = [];
for i=1:n_diff+1
    for j=1:n_row
        raw_i = subs( jacobian(Q_d(i),a(j,:)), t, 0);
        raw_f = subs( jacobian(Q_d(i),a(j,:)), t, T);
        A_i = [A_i;raw_i];
        A_f = [A_f;raw_f];
    end
end
A = [A_i;A_f];
B = [q_i,q_f]'

sol = linsolve(A, B);

traj = vpa(subs(Q_d,a',sol),2);




end