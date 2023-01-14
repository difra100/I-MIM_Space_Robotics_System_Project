function q_f = get_target_conf(target,q_bounds, init_guess, l, L_s)

% Formulation of a constrained non linear optimization problem, to solve
% the planet pointing issue.
disp('Getting the solution to the problem of inverse kinematics:')

%% Constrained Solution
q_1 = optimvar('q_1',1,'LowerBound',q_bounds.low(1),'UpperBound',q_bounds.high(1)); 
q_2 = optimvar('q_2',1,'LowerBound',q_bounds.low(2),'UpperBound',q_bounds.high(2));

qq = [q_1;q_2];
[DHTABLE,T_i_b]= DH_generator(l,qq, L_s);
[T_EE,R_EE,f] = forward_kinematics(DHTABLE,T_i_b);
T_tip = T_EE*trvec2tform([1,0,0]);
f_tip = T_tip(1:3,4);



v_1 = target - f;       % direction from EE to target
v_2 = target - f_tip;         % direction of the antenna

obj = norm(cross(v_1,v_2))^2;

% The objective formulation is better explained in the report

prob = optimproblem("Objective", obj, 'ObjectiveSense', 'min');
prob.Constraints.c1 = -v_2'*v_1 <= 0; 
q0.q_1 = init_guess(1);
q0.q_2 = init_guess(2);
[q_s,fval] = solve(prob,q0);


if q_s.q_1 <= q_bounds.high(1) && q_s.q_2 <= q_bounds.high(2) && q_s.q_1>= q_bounds.low(1) && q_s.q_2 >= q_bounds.low(2)
    q_f = [q_s.q_1; q_s.q_2];
    fprintf('\nOne feasible solution has been found: [%f, %f]\n',q_f')
    fprintf('with a confidence of: +/- %f\n',fval)
else 
    disp('No feasible solution has been found!')
    q_f = init_guess;




end