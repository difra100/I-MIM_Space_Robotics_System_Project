function [M, V,B,C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2)

%% Explainations
% INPUTS:
%   - q = [q1,q2] 
%   - dq = [dq1, dq2]
%   - ddq = [ddq1,ddq2]
%   - m = [m1,m2] masses
%   - l = [l1,l2] link lenghts
%   - d = [d1,d2] CoMs positions
%   - I = [I1, I2] inertia coupling matrics

%% KINEMATIC ENERGY

% link 1
pc1 =[d(1)*cos(q(1)) d(1)*sin(q(1)) 0]';
vc1 = diff(pc1,q(1))*dq(1);
Tl1 = (1/2)*m(1)*vc1'*vc1;

om1 = [0 0 dq(1)]';
Ta1 = (1/2)*om1'*diag([I1(1) I1(2) I1(3)])*om1;

T1= simplify(Tl1+Ta1);

% link2
pc2 = [l(1)*cos(q(1))+d(2)*cos(q(1)+q(2)) l(1)*sin(q(1))+d(2)*sin(q(1)+q(2)) 0]';
vc2 = simplify(diff(pc2,q(1))*dq(1)+diff(pc2,q(2))*dq(2));
T2l = simplify((1/2)*m(2)*vc2'*vc2);

om2=[0 0 dq(1)+dq(2)]';

T2a=simplify((1/2)*om2'*diag([I2(1) I2(2) I2(3)])*om2);

T2=simplify(T2l+T2a);

% total kinetic energy
T=T1+T2;
T=simplify(T1+T2);


% collect in base of term that you pass it in this case, collect terms that has dq1^2 and 
% do the same for dq2^2 because you need them to compute M
T=collect(T,dq(1)^2);
T=collect(T,dq(2)^2);


%% INERTIA MATRIX (M)

% we extract the element M(1,1) from the total kinetic energy by doing twice the derivative
% respect qd1 because this will be the factor that multiplyin the square of velocity  of joint 1
% we do the same also for joint 2
% the factor 1/2 disappear automatically in this differentation

M(1,1)=diff(T,dq(1),2);
M(2,2)=diff(T,dq(2),2);

TempB1=diff(T,dq(1));

M(1,2)=diff(TempB1,dq(2));
M(2,1)=M(1,2);

M=simplify(M);


%% CORIOLIS/CENTRIFUGAL TERM

% Getting the Christoffel matrices
M1=M(:,1);
M2=M(:,2);
C1=(1/2)*(jacobian(M1,q)+jacobian(M1,q)'-diff(M,q(1)));
C2=(1/2)*(jacobian(M2,q)+jacobian(M2,q)'-diff(M,q(2)));

% Getting the final terms just by multipling per the velocities
dq=[dq(1);dq(2)];
V1=dq'*C1*dq;
V2=dq'*C2*dq;
V=simplify([V1;V2]);


%% CONFIGURATION SPACE FORM (splitting c in B,C)
B = diff(diff(V,dq(1)),dq(2));
C = [diff(diff(V,dq(1)),dq(1)),diff(diff(V,dq(2)),dq(2))];



%% POTENTIAL ENERGY ?

% Gravitational accelerations are assumed to be zero in space.


end
 