function q_f = get_target_conf(target,q_bounds, init_guess, l)

% Get the paramentric espression of the line interpolating EE position and
% the target:
% rect = (x - x_p)/v_1 - (y - y_p)/v_2 = 0 and (y - y_p)/v_2 - (z - z_p)/v_3 = 0
% Imposing x,y,z = x_tip,y_tip,z_tip (belongs to the rect)
% Looking for matricial form:

disp('Finding the solution...')

%% Constrained Solution
q_1 = optimvar('q_1',1,'LowerBound',q_bounds.low(1),'UpperBound',q_bounds.high(1)); % 3-by-3 variable
q_2 = optimvar('q_2',1,'LowerBound',q_bounds.low(2),'UpperBound',q_bounds.high(2));

qq = [q_1;q_2];
[DHTABLE,T_i_b]= DH_generator(l,qq)
[T_EE,R_EE,f] = forward_kinematics(DHTABLE,T_i_b);
T_tip = T_EE*trvec2tform([1,0,0]);
f_tip = T_tip(1:3,4);



v_1 = target - f;       % direction from EE to target
v_2 = target - f_tip;         % direction of the antenna

obj = norm(cross(v_1,v_2))^2 %+ 10*(norm(v_2)/0.001*norm(v_1));
prob = optimproblem("Objective", obj, 'ObjectiveSense', 'min');
prob.Constraints.c1 = -v_2'*v_1 <= 0; 
q0.q_1 = init_guess(1);
q0.q_2 = init_guess(2);
[q_s,fval] = solve(prob,q0);


%% UNCONSTRAINED SOLUTION

% options = optimoptions('fminunc','Display','final','Algorithm','quasi-newton');
% q_f = init_guess;
% in = init_guess;
% 
% fh2 = matlabFunction(obj,'vars',{q}); 
% % fh2 = objective with no gradient or Hessian
% [q_f,fval,exitflag,output2] = fminunc(fh2,init_guess,options);




if q_s.q_1 <= q_bounds.high(1) && q_s.q_2 <= q_bounds.high(2) && q_s.q_1>= q_bounds.low(1) && q_s.q_2 >= q_bounds.low(2)
    disp('The solution is feasible..........')
    q_f = [q_s.q_1; q_s.q_2]
    fval = fval
else 
    disp('No good solution')
    q_f = init_guess;




end