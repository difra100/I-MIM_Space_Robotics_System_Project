function [J_L,J_A] = geometric_jacobian (DHtable,q,sequence)

% INPUT:
%   - DHtable: DH table 
%   - syms: symbolic variables for joints [q1,q2,q3...qN]
%   - sequence = sequence of joints 'r' revolut and 's' prismatic 
%       (eg. ['r','r','r','r','r','r','r']

% OUTPUT: Linear and angular jacobians 
%         (which stacked compose the geometric jacobian)

    n_j = size(DHtable,1);      % number of joints
    if (n_j ~= size(sequence))
        errorID = 'myComponent:inputError';
        msg = 'the number of joints in the sequence and the number the number in the table are different!';
        exce = MException(errorID,msg);
        throw(exce)
    end
    % -----> angular jacobian
    J_A = [];               
    R = eye(3);
    T = eye(4);
    for i=1:n_j
        line = DHtable(i,:);
        A_i = DHMatrix(line);
        if sequence(:,i)== 'r'
            z = R*[0;0;1];
            J_A = [J_A z];
        else
            J_A = [J_A [0;0;0]];
        end
        R = R*A_i(1:3,1:3);
        T = T*A_i;
    end
    
    % ----> linear jacobian
    p = T(1:3,4);
    J_L = [];
    for i=1:n_j
        J_L_i = simplify(diff(p,q(i)));
        J_L = [J_L, J_L_i];

    
end