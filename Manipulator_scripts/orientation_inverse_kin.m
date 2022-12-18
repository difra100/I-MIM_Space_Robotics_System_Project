function target_conf = orientation_inverse_kin(angles,q,R_EE)
angles(3)=0;
R = rpy_rotation('xyz',angles);


disp(vpa(R_EE,2))
%disp(R_EE(3,2)) %-cos(q1)
%disp(R_EE(1,3)) %-cos(q2)
%disp(R_EE(1,1)) %-sin(q2)
disp(vpa(R_EE(2,3),2)) %-cos(q2)*sin(q1)
disp(R)

cos_q1 = -R(3,2)

cos_q2 = -R(1,3)
sin_q2 = -R(1,1)
sin_q1 = -R(2,3)/cos_q2
q1_f = atan2(sin_q1,cos_q1)
q2_f = atan2(sin_q2,cos_q2)
target_conf = [q1_f;q2_f];
end