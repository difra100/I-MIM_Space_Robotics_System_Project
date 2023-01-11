function [qs,dqs,ddqs, points] = get_trajectory_points(traj_q,traj_dq, ...
                                                traj_ddq,t,time_steps, q,f)

count = length(time_steps); % discrete time intervals

 % preallocate memory
qs = zeros(count, size(q,1)); 
dqs = zeros(count, size(q,1)); 
ddqs = zeros(count, size(q,1)); 
points = zeros(count, 3);


for i = 1:count
    qs(i,:) = subs(traj_q,t,time_steps(i));
    
    dqs(i,:) = subs(traj_dq,t,time_steps(i));
    ddqs(i,:) = subs(traj_ddq,t,time_steps(i));
    points(i,:) = subs(f,q',qs(i,:));
    
end




end






















