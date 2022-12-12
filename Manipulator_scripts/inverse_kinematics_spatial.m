function [q] = inverse_kinematics(p,l)
    % closed form solution of the inverse kinematics for 2R robots
    % INPUT:    p =[p_x, p_y]: position in the Cartesian space 
    %           l = [l_1, l_2]: link lengths
    % OUTPUT:   q2_g, q1_g position in the Joint space
    p_x = p(1);
    p_y = p(2);
    l_1 = l(1);
    l_2 = l(2);

    c2 = (p_x^2+p_y^2-(l_1^2+l_2^2))/(2*l_1*l_2);
    s2 = -sqrt(1-c2^2); 
    % choose the minus sign to have the elbow-up solution as goal
    % being this ‘closer’ to the start configuration qs
    
    q1_g = atan2(p_y*(l_1+l_2*c2)-p_x*l_2*s2,p_x*(l_1+l_2*c2)+p_y*l_2*s2);
    q2_g = atan2(s2,c2);
    q = [q1_g,q2_g];
end