function [qs, points] = get_trajectory_points(traj, t, T, q,f)







time_steps = (0:0.2:T)'; % Time
count = length(time_steps); % discrete time intervals

 % preallocate memory
qs = zeros(count, 2); 
points = zeros(count, 3);
q1 = q(1);
q2 = q(2);

for i = 1:count
    qs(i,:) = subs(traj,t,time_steps(i));
    points(i,:) = vpa(subs(subs(f,q1,qs(i,1)),q2,qs(i,2)),2);
end

end






















