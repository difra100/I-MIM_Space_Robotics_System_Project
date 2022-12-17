
function robot = create_robot(DHparam,q_i)

robot = rigidBodyTree('DataFormat','col');

col1 = DHparam(:,1);
col2 = DHparam(:,2);
col3 = DHparam(:,3);
col4 = DHparam(:,4);
DHparam = [col2 col1 col3 col4];
dx=2;
dy=2;
dz=2;

T_i_b = [0 1 0 dx;-1 0 0 dy;0 0 1 dz;0 0 0 0];

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1,DHparam(1,:),'dh');
%setFixedTransform(joint1, trvec2tform([7,0,0]));
joint1.HomePosition = q_i(1);
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2,DHparam(2,:),'dh');
%setFixedTransform(joint2, trvec2tform([5,0,0]));
joint2.HomePosition = q_i(2);
joint2.JointAxis = [0 0 1];

body2.Joint = joint2;
addBody(robot, body2, 'link1');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([0,0,0]));
body.Joint = joint;
addBody(robot, body, 'link2');

showdetails(robot)

end


