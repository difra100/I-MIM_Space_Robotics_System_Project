function time_scaling = timing_law(q,s,path,tau_m,tau_max)

% Timing law computation
% The idea is that you get the torque erogated by
% the motors (as a time series), get the max value, and consequently the violation (with
% respect to the boundaries), compute the necessary time scaling and finally 
% get the timing law by tuning the total time of motion (T).

time_steps = (0:0.02:1)'; % Time
count = length(time_steps); % discrete time intervals
tau_val_best = [0,0];

for i = 1:count
   s_i = time_steps(i);
   q_i = subs(path,s,s_i)';
   tau_vals = subs(tau_m,q',q_i);
   tau_vals = subs(tau_vals,s,s_i);
   for j=1:2
       if abs(tau_vals(j))>tau_val_best(j) 
           tau_val_best(j) = abs(tau_vals(j));
       end
   end
end

% Let's get the time scaling factor necessary to saturate the joints'torque to
% the boundaries (the most violated), which would be used to compute the timing law
scaling = max([tau_val_best(1)/tau_max(1), ...
                    tau_val_best(2)/tau_max(2)]);
time_scaling = sqrt(scaling);
    

% Define the timimg laws (s(t))
 
end
