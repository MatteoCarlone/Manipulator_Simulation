%% GetBasicVectorWrtBase function 
% inputs :
%   - biTei: vector of matrices containing the transformation matrices from link i to link i +1. The
%            size of biTri is equal to (4,4,linkNumber)
%   - linkNumber: number of link for which computing the basic vector from the
%            base to the joint
% output:
%   - r : basic vector from the base to the input joint 

function [r]=GetBasicVectorWrtBase(biTei, linkNumber)

    % Get the transormation matrix of the frame i with respect to
    % the base frame 
    T = GetTransformationWrtBase(biTei, linkNumber);

    % taking only the first three rows of the fourth column
    r = T( (1 : 3), 4);

end