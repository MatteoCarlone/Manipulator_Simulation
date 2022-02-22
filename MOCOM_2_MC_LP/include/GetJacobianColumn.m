%% Inverse Kinematic Function
% Function computing the end effector jacobian column for the input
% parameters.
%
% Inputs
% - bTei: transformation matrix from the base to the ith joint for the
%   current configuration
% - bTe: tranformation matrix from the base to the end effector
% - jointType: 0 if the joint is revolute,1 if the joint is prismatic (is
%   referred to the joint corresponding to bTe).
%
% Output
% - h: Jacobian column h(1:3) angular part, h(4:6) linear

function [h] = GetJacobianColumn(bTei, bTe, jointType)
    %Versor of translation or rotation
    ki = bTei( (1 : 3) , 3);
    rei = bTe(1:3, 4) - bTei(1:3, 4);
    
    if(jointType == 1)%Prismatic
       %Jai is a null vector
       Jai = [0; 0; 0];
       %The traslation of the end effector is along ki
       Jli = ki;
    end
    
    if(jointType == 0)%Revolute
       %The rotation is around ki
       Jai = ki;
       %The transl. of the e.e. is along a versor given by the cross product of ki and rei 
       Jli = cross(ki, rei);
    end
    
    h = [Jai; Jli];
end
