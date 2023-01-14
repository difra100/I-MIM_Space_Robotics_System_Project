function q_f = get_target_conf_VM(target, q_bounds, init_guess, l_0,l)

% Get the paramentric espression of the line interpolating EE position and
% the target

disp('Finding the solution...')

%% Constrained Solution

% To avoid getting any divergency in the solution, it has been introduced some
% artificial boundaries to the fictitious satellite's attitude's angles
% theta1, theta2 and theta3.
theta1 = optimvar('theta1', 1, 'LowerBound',-2*pi, 'UpperBound',2*pi);
theta2 = optimvar('theta2', 1, 'LowerBound',-2*pi, 'UpperBound',2*pi);
theta3 = optimvar('theta3', 1, 'LowerBound',-2*pi, 'UpperBound',2*pi);
q_1 = optimvar('q_1',1,'LowerBound',q_bounds.low(1),'UpperBound',q_bounds.high(1)); % 3-by-3 variable
q_2 = optimvar('q_2',1,'LowerBound',q_bounds.low(2),'UpperBound',q_bounds.high(2));

Q = [theta1; theta2; theta3; q_1; q_2];
   
DHtable = DH_generator_VM(double(l_0),double(l), Q);
[T_EE,~, p_EE,~,~] = forward_kinematics_VM(DHtable);

T_tip = T_EE*trvec2tform([1,0,0]);
f_tip = T_tip(1:3,4);
f = p_EE;


v_1 = target - f;       % direction from EE to target
v_2 = target - f_tip;         % direction of the antenna

obj = norm(cross(v_1,v_2))^2;

% The objective formulation is better explained in the report

prob = optimproblem("Objective", obj, 'ObjectiveSense', 'min');
prob.Constraints.c1 = -v_2'*v_1 <= 0; 

q0.theta1 = init_guess(1);
q0.theta2 = init_guess(2);
q0.theta3 = init_guess(3);

q0.q_1 = init_guess(4);
q0.q_2 = init_guess(5);
[q_s,fval] = solve(prob,q0);


if q_s.q_1 <= q_bounds.high(1) && q_s.q_2 <= q_bounds.high(2) && q_s.q_1>= q_bounds.low(1) && q_s.q_2 >= q_bounds.low(2)
    disp('The solution is feasible..........')
    q_f = [q_s.theta1; q_s.theta2; q_s.theta3; q_s.q_1; q_s.q_2];
    fprintf(' - Solution: [%f %f %f] \n', q_f)
    fprintf(' - Objective function: %f \n', fval)
    
else 
    disp('No good solution')
    q_f = init_guess;
end