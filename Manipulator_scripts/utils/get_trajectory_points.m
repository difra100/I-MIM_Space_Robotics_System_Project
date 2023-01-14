function [qs,dqs,ddqs, points] = get_trajectory_points(traj_q,traj_dq, ...
                                                traj_ddq,t,time_steps, q,f)
% This function represent just an util to get the trajectory with the
% configuration values substituted within.
% INPUTs: traj_q/dq/ddq are the parametrized function of joints' positon, velocity
% and acceleration respectively, t: is the symbolic time variable, time
% steps: It is the time trajectory, once it has been discretized according
% to the sampling time, f: This is the EE position expressed as a function
% of q.
% OUTPUTs: Substituted trajectories (qs, dqs, ddqs) of the joint positions, velocities and
% acceleration evolving over time, together with their EE position (points).

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






















