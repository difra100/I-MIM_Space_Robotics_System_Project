function robot = create_robot_VM(l_0,l,q_i)

robot = rigidBodyTree('DataFormat','col'); %The 'base' is actually the inertia frame


[DHparam] = DH_generator_VM(l_0,l,q_i);
col1 = DHparam(:,1);
col2 = DHparam(:,2);
col3 = DHparam(:,3);
col4 = DHparam(:,4);
DHparam = [col2 col1 col3 col4];
% This is becaue rigid body uses a different order in the 
% definition of the DH table
% Definition of the base RF


% Defining the (3) spacecraft link-(spherical)joint

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1,double(DHparam(1,:)),'dh');
joint1.HomePosition = q_i(1);
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2,double(DHparam(2,:)),'dh');
joint2.HomePosition = q_i(1);
joint2.JointAxis = [0 0 1];
body2.Joint = joint2;
addBody(robot, body2, 'link1');

body3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(joint3,double(DHparam(3,:)),'dh');
joint3.HomePosition = q_i(1);
joint3.JointAxis = [0 0 1];
body3.Joint = joint3;
addBody(robot, body3, 'link2');



% Defining the 1° link-joint
body4 = rigidBody('link4');
joint4 = rigidBodyJoint('joint4', 'revolute');
setFixedTransform(joint4,double(DHparam(4,:)),'dh');
joint4.HomePosition = q_i(1);
joint4.JointAxis = [0 0 1];
body4.Joint = joint4;
addBody(robot, body4, 'link3');

% Defining the 2° link-joint
body5 = rigidBody('link5');
joint5 = rigidBodyJoint('joint5', 'revolute');
setFixedTransform(joint5,double(DHparam(5,:)),'dh');
joint5.HomePosition = q_i(5);
joint5.JointAxis = [0 0 1];
body5.Joint = joint5;
addBody(robot, body5, 'link4');

% Defining the EE
tool = rigidBody('tool');
jointEE = rigidBodyJoint('fix1','fixed');
setFixedTransform(jointEE, trvec2tform([0.01,0,0]));
tool.Joint = jointEE;
addBody(robot, tool, 'link5');

showdetails(robot)

end