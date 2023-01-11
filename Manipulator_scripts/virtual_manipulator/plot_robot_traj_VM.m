function plot_robot_traj_VM(robot, q_d, points,target_pos,Q,f,f_tip)


%robot = create_robot(DH_T1,DH_T2);

figure
show(robot,q_d(1,:)');

axis([-1,1,-1,1,-1,1]*30);


%------> Target in cartesia space = SPHERE
hold on
plot3(points(:,1),points(:,2),points(:,3),'k')


%------> Antenna
hold on
EE_pos = subs(f,Q,q_d(1,:)');
scatter3(EE_pos(1),EE_pos(2),EE_pos(3),10,'r','filled');

hold on
tip_pos = subs(f_tip,Q,q_d(1,:)');
scatter3(tip_pos(1),tip_pos(2),tip_pos(3),10,'b','filled');

%------> Target (planet) = FULL SPHERE
hold on
scatter3(target_pos(1),target_pos(2),target_pos(3),100,'g','filled');


%-------> Trajectory
framesPerSecond = 30;
r = rateControl(framesPerSecond);
for i = 1:size(q_d,1)
    show(robot,q_d(i,:)','PreservePlot',false);
    drawnow
    EE_pos = subs(f,Q,q_d(i,:)');
    scatter3(EE_pos(1),EE_pos(2),EE_pos(3),20,'r','filled');
  
    tip_pos = subs(f_tip,Q,q_d(i,:)');
    scatter3(tip_pos(1),tip_pos(2),tip_pos(3),10,'b','filled');

    waitfor(r);
end


