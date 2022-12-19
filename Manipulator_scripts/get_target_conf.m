function q_f = get_target_conf(q,f,f_tip,target,q_bounds)


% Get the paramentric espression of the line interpolating EE position and
% the target. Then you define the rect that interpolate p_EE and target:
% rect = (x - x_p)/l - (y - y_p)/m = 0 and (y - y_p)/m - (z - z_p)/n = 0
% Imposing x,y,z = x_tip,y_tip,z_tip
% Looking for matricial form:

% Solution model based
% v = [];
% for i=1:3
%     v = [v;f(i)-target(i)];
% end
% 
% rect_system = [(f_tip(1)-target(1))/v(1)-(f_tip(2)-target(2))/v(2)==0;
%                 (f_tip(1)-target(1))/v(1)-(f_tip(3)-target(3))/v(3)==0];
%                 %sign(n_1-n_2)==-1];
% 
% [q1_f,q2_f] = vpasolve(rect_system,[q(1),q(2)],[0,0]);
% q_f = [q1_f;q2_f];


% Solution as optimizaion problem (NOT WORK)

% q1 = optimvar('q1','LowerBound',q_bounds.low(1),'UpperBound',q_bounds.high(1));
% q2 = optimvar('q2','LowerBound',q_bounds.low(2),'UpperBound',q_bounds.high(2));
% q = [q1,q2];
% [DHTABLE,T_i_b]= DH_generator([7,5],q);
% T = DHMatrix(DHTABLE);
% T_EE = T*T_i_b;
% p_EE = T_EE(1:3,4);
% T_tip = T_EE*trvec2tform([1,0,0]);
% p_tip = T_tip(1:3,4);
% v = [];
% for i=1:3
%     v = [v;p_EE(i)-target(i)];
% end
% error_1 = (p_tip(1)-target(1))/v(1)-(p_tip(2)-target(2))/v(2);
% error_2 = (p_tip(1)-target(1))/v(1)-(p_tip(3)-target(3))/v(3);
% 
% n_1 = 0;
% n_2 = 0;
% for i=1:3
%     n_1 = n_1+(f_tip(i)-target(i))^2;
%     n_2 = n_2+(f(i)-target(i))^2;
% end
% n_1 = sqrt(n_1);    % distance Tip->target
% n_2 = sqrt(n_2);    % distance EE->trarget
% 
% % Initial Guesses
% q0.q1 = 0;
% q0.q2 = 0;
% 
% prob = optimproblem;
% prob.Objective = error_1^2 + error_2^2;
% 
% % %joint boundaries
% % prob.Constraints.cons1 = -q1 <= -q_bounds.low(1);
% % prob.Constraints.cons2 = q1 <= q_bounds.high(1);
% % prob.Constraints.cons3 = -q2 <= -q_bounds.low(2);
% % prob.Constraints.cons4 = q2 <= q_bounds.high(2);
% 
% % The solution we want is: distance EE->trarget > Tip->target
% %prob.Constraints.cons = sqrt(n_1)-sqrt(n_2)<= 0;
% 
% q_f = solve(prob,q0);
% 
% q_f = [q_f.q1;q_f.q2];

end