function plot_robot_traj(robot,traj,q,f,t,T)

q1 = q(1);
q2 = q(2);
% 
% [DH_T1,A1] = DHMatrix( DHparam(1,:));
% [DH_T2,A2] = DHMatrix( DHparam(2,:));


%robot = create_robot(DH_T1,DH_T2);


time_steps = (0:0.2:T)'; % Time
count = length(time_steps); % discrete time intervals

 % preallocate memory
qs = zeros(count, 2); 
points = zeros(count, 3);

for i = 1:count
    qs(i,:) = subs(traj,t,time_steps(i));
    points(i,:) = vpa(subs(subs(f,q1,qs(i,1)),q2,qs(i,2)),2);
end

figure
show(robot,qs(1,:)');
axis([-1,1,-1,1,-1,1]*30)
%view(2)
%ax = gca;
%ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3),'k')

framesPerSecond = 30;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end