function q_f = get_target_conf(q,f,f_tip,target,q_bounds, init_guess)

% Get the paramentric espression of the line interpolating EE position and
% the target:
% rect = (x - x_p)/v_1 - (y - y_p)/v_2 = 0 and (y - y_p)/v_2 - (z - z_p)/v_3 = 0
% Imposing x,y,z = x_tip,y_tip,z_tip (belongs to the rect)
% Looking for matricial form:


v_1 = target' - f;       % direction from EE to target
v_2 = target' - f_tip;         % direction of the antenna

obj = 0.0001*norm(cross(v_1,v_2))^2 + 10*(norm(v_2)/0.001*norm(v_1));

options = optimoptions('fminunc','Display','final','Algorithm','quasi-newton');
q_f = init_guess;
in = init_guess;
while norm(q_f - in) < 0.1


    fh2 = matlabFunction(obj,'vars',{q}); 
    % fh2 = objective with no gradient or Hessian
    [q_f,fval,exitflag,output2] = fminunc(fh2,init_guess,options);

    if norm(q_f - in) < 0.1
        init_guess = init_guess + [0.01;-0.01];
    end
end


if q_f(1) <= q_bounds.high(1) && q_f(2) <= q_bounds.high(2) && q_f(1)>= q_bounds.low(1) && q_f(2) >= q_bounds.low(2)
    disp('The solution is feasible..........')
else 
    disp('No good solution')


end