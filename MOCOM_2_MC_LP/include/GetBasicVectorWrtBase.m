
%% GetBasicVectorWrtBase function 
% inputs :
% - biTei: vector of matrices containing the transformation matrices from link i to link i +1. The
% size of biTri is equal to (4,4,numberOfLinks)
% - linkNumber: number of link for which computing the basic vector from the
% base to the joint
% output:
% r : basic vector from the base to the input joint 

function [r]=GetBasicVectorWrtBase(biTei, linkNumber)

%get the transormation matrix of the frame i wrt frame 0
T = GetTransformationWrtBase(biTei, linkNumber);

%take only the first three rows of the fourth column
r = T( (1 : 3), 4);

end