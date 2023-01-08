function q_f = get_target_conf_VM(target, q_bounds, init_guess, l, m, m_s)

% Get the paramentric espression of the line interpolating EE position and
% the target

disp('Finding the solution...')

%% Constrained Solution
theta1 = optimvar('theta1', 1);
theta2 = optimvar('theta2', 1);
theta3 = optimvar('theta3', 1);
q_1 = optimvar('q_1',1,'LowerBound',q_bounds.low(1),'UpperBound',q_bounds.high(1)); % 3-by-3 variable
q_2 = optimvar('q_2',1,'LowerBound',q_bounds.low(2),'UpperBound',q_bounds.high(2));

qq = [theta1; theta2; theta3; q_1; q_2];
[VM_direct_kinematics,T_lvlh_b, T_EE] = forward_kinematics_VM(l, qq(4:5), qq(1:3), true, m, m_s)
T_tip = T_EE*trvec2tform([1,0,0]);
f_tip = T_tip(1:3,4);
f = VM_direct_kinematics;


v_1 = target - f;       % direction from EE to target
v_2 = target - f_tip;         % direction of the antenna

obj = norm(cross(v_1,v_2))^2 %+ 10*(norm(v_2)/0.001*norm(v_1));

prob = optimproblem("Objective", obj, 'ObjectiveSense', 'min');
prob.Constraints.c1 = -v_2'*v_1 <= 0; 

q0.theta1 = init_guess(1);
q0.theta2 = init_guess(2);
q0.theta3 = init_guess(3);

q0.q_1 = init_guess(4);
q0.q_2 = init_guess(5);
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
    q_f = [q_s.theta1; q_s.theta2; q_s.theta3; q_s.q_1; q_s.q_2]
    fval = fval
else 
    disp('No good solution')
    q_f = init_guess;




end