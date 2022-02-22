%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - bTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

function J = GetJacobian(bTei, bTe, jointType)

    numLinks = length(jointType);
    J = zeros(6, numLinks);

    for i = 1 : numLinks
        %Compute the trasmormation matrix wrt the base to pass it to GetJacobianColumn 
        bTi = GetTransformationWrtBase(bTei, i);
        %Call GetJacobianColumn for each joint
        J(:, i) = GetJacobianColumn(bTi, bTe, jointType(i));
    end

end










