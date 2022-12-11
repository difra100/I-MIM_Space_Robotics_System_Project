function robot = create_robot(T1,T2)

robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, T1);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, T2);

body.Joint = joint;
addBody(robot, body, 'link2');

end