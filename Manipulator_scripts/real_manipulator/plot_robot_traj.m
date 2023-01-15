function plot_robot_traj(robot, q_d, points,target_pos,q,f,f_tip)

q1 = q(1);
q2 = q(2);
% 
% [DH_T1,A1] = DHMatrix( DHparam(1,:));
% [DH_T2,A2] = DHMatrix( DHparam(2,:));
target_pos = target_pos;

%robot = create_robot(DH_T1,DH_T2);

figure
show(robot,q_d(1,:)');

axis([-1,1,-1,1,-1,1]*10);

xlabel('Y')
ylabel('X')
%view(2)
%ax = gca;
%ax.Projection = 'orthographic';

%------> Trarget in cartesia space = SPHERE
hold on
plot3(points(:,1),points(:,2),points(:,3),'k')

%-------> SpaceCraft = CUBE
hold on
dir = -pi : pi/2 : pi;                       % Define Corners                                         
side = 1;                   % Define Angular Orientation (1pi/4ase0)
x_cube = [cos(dir+pi/4); cos(dir+pi/4)]/cos(pi/4)*side;
y_cube = [sin(dir+pi/4); sin(dir+pi/4)]/sin(pi/4)*side;
z_cube = [-ones(size(dir)); ones(size(dir))]*side;
surf(x_cube, y_cube, z_cube, 'FaceColor','w');  % Plot Cube
patch(x_cube', y_cube', z_cube', 'w');

%------> Antenna
hold on
EE_pos = subs(f,q,q_d(1,:)');
scatter3(EE_pos(1),EE_pos(2),EE_pos(3),5,'r','filled');

hold on

tip_pos = subs(f_tip,q,q_d(1,:)');
scatter3(tip_pos(1),tip_pos(2),tip_pos(3),2,'b','filled');

%------> Target (planet) = FULL SPHERE
hold on
scatter3(target_pos(1),target_pos(2),target_pos(3),100,'g','filled');


%-------> Trajectory
framesPerSecond = 30;
r = rateControl(framesPerSecond);
for i = 1:size(q_d,1)
    show(robot,q_d(i,:)','PreservePlot',false);
    drawnow
    EE_pos = subs(f,q,q_d(i,:)');
    scatter3(EE_pos(1),EE_pos(2),EE_pos(3),20,'r','filled');
  
    tip_pos = subs(f_tip,q,q_d(i,:)');
    scatter3(tip_pos(1),tip_pos(2),tip_pos(3),10,'b','filled');

    waitfor(r);
end


