function q_f = get_target_conf(q,f,f_tip,target,q_bounds)

% Get the paramentric espression of the line interpolating EE position and
% the target:
% rect = (x - x_p)/v_1 - (y - y_p)/v_2 = 0 and (y - y_p)/v_2 - (z - z_p)/v_3 = 0
% Imposing x,y,z = x_tip,y_tip,z_tip (belongs to the rect)
% Looking for matricial form:

% Solution model based
v = [];
for i=1:3
    v = [v;target(i)-f(i)];     % pointing direction
end

rect_system = [(target(1)-f_tip(1))/v(1)-(target(2)-f_tip(2))/v(2)==0;
                (target(1)-f_tip(1))/v(1)-(target(3)-f_tip(3))/v(3)==0];
          
[q1_f,q2_f] = vpasolve(rect_system,[q(1),q(2)],[0,0]);
q_f = [q1_f;q2_f];


% If v is given and no target point available:
%rect_system = [(f_tip(1)-f(1))/v(1)-(f(2)-f(2))/v(2)==0;
%                (f_tip(1)-f(1))/v(1)-(f(3)-f(3))/v(3)==0];








% Solution as optimization problem (NOT WORK)
% q_i = [0;0];
% q1 = optimvar('q1','LowerBound',q_bounds.low(1),'UpperBound',q_bounds.high(1));
% q2 = optimvar('q2','LowerBound',q_bounds.low(2),'UpperBound',q_bounds.high(2));
% q = [q1,q2];
% 
% [DHTABLE,T_i_b]= DH_generator([7,5],q);
% 
% T = DHMatrix(DHTABLE);            % base -> EE
% T_EE = T*T_i_b;                   % inertia -> EE
% p_EE = T_EE(1:3,4);             % forwrd kinematics
% 
% T_tip = T_EE*trvec2tform([1,0,0]);     % inertia -> tip
% p_tip = T_tip(1:3,4);              % forward kinematics tip
% 
% v_1 = target' - p_EE;       % direction from EE to target
% v_2 = p_tip - p_EE;         % direction of the antenna
% 
% % Initial Guesses
% q0.q1 = 0;
% q0.q2 = 0;
% 
% % the objective is to make v_1 and v_2 parallel ==> cross() = [0,0,0]
% obj = norm(cross(v_1,v_2));
% prob = optimproblem('Objective',obj);
% 
% % The solution we want is the one such that:
% %   distance EE->trarget > Tip->target
% 
% prob.Constraints.constraint = dot(v_1,v_2)/(norm(v_1)*norm(v_2)) == 1;
% 
% show(prob)
% 
% q_f = solve(prob,q0);
% 
% q_f = [q_f.q1;q_f.q2];

end