
function robot = create_robot(l,q_i)

robot = rigidBodyTree('DataFormat','col'); %The 'base' is actually the inertia frame


[DHparam,T_i_b] = DH_generator(l,q_i);
col1 = DHparam(:,1);
col2 = DHparam(:,2);
col3 = DHparam(:,3);
col4 = DHparam(:,4);
DHparam = [col2 col1 col3 col4];
% This is becaue rigid body uses a different order in the 
% definition of the DH table

% Definition of the base RF 
body0 = rigidBody('base_fixed');
transf = rigidBodyJoint('transf', 'fixed');
setFixedTransform(transf, T_i_b);
body0.Joint = transf;
addBody(robot, body0, 'base');

% Defining the 1° link-joint
body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1,DHparam(1,:),'dh');
joint1.HomePosition = q_i(1);
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot, body1, 'base_fixed');

% Defining the 2° link-joint
body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2,DHparam(2,:),'dh');
joint2.HomePosition = q_i(2);
joint2.JointAxis = [0 0 1];
body2.Joint = joint2;
addBody(robot, body2, 'link1');

% Defining the EE
tool = rigidBody('tool');
jointEE = rigidBodyJoint('fix1','fixed');
setFixedTransform(jointEE, trvec2tform([0,0,0]));
body.Joint = jointEE;
addBody(robot, tool, 'link2');

% showdetails(robot)

end


