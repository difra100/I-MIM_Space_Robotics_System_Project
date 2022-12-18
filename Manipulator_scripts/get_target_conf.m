function q_f = get_target_conf(q,q_i,f,f_tip,target)


% Get the paramentric espression of the line interpolating EE position and
% the target. Then you define the rect that interpolate p_EE and target:
% rect = (x - x_p)/l - (y - y_p)/m = 0 and (y - y_p)/m - (z - z_p)/n = 0
% Imposing x,y,z = x_tip,y_tip,z_tip
% Looking for matricial form:

% Solution model based

v = [];
for i=1:3
    v = [v;f(i)-target(i)];
end

rect_system = [(f_tip(1)-target(1))/v(1)-(f_tip(2)-target(2))/v(2)==0;
                (f_tip(1)-target(1))/v(1)-(f_tip(3)-target(3))/v(3)==0];
                %sign(n_1-n_2)==-1];

[q1_f,q2_f] = vpasolve(rect_system,[q(1),q(2)],[0,0]);
q_f = [q1_f;q2_f];


% Solution as optimizaion problem (can add also the constraints)
% prob = optimproblem;
% x = optimvar('x',2);
% q = [x(1);x(2)]
% 
% [DHTABLE,T_i_b]= DH_generator([7,5],q);
% T = DHMatrix(DHTABLE);
% T_EE = T*T_i_b;
% f = T_EE(1:3,4)
% T_tip = T_EE*trvec2tform([1,0,0]);
% p_tip = T_tip(1:3,4);
% 
% prob.Constraints.const1 = (f_tip(1)-target(1))/v(1)-(f_tip(2)-target(2))/v(2)==0;
% prob.Constraints.const2 = (f_tip(1)-target(1))/v(1)-(f_tip(3)-target(3))/v(3)==0;
% 
% n_1 = 0;
% n_2 = 0;
% for i=1:3
%     n_1 = n_1+(f_tip(i)-target(i))^2;
%     n_2 = n_2+(f(i)-target(i))^2;
% end
% n_1 = sqrt(n_1);
% n_2 = sqrt(n_2);
% 
% prob.Constraints.const3 = sign(n_1-n_2)==-1;



end