%% GetFrameWrtFrame function 
% inputs: 
% - linkNumber_i : number of ith link 
% - linkNumber_j: number of jth link 
% - biTei: vector of matrices containing the transformation matrices from link i to link i+1 for the current q.
% output:
% iTj : transformationMatrix from link i and link j for the configuration
% described in biTei.

function [iTj]=GetFrameWrtFrame(linkNumber_i, linkNumber_j,biTei)

 %multiply the inverse of the transformation matrix of the frame i wrt base
 %by the transformation matrix of the frame j wrt base
 iTj = GetTransformationWrtBase(biTei, linkNumber_i)^-1 * GetTransformationWrtBase(biTei, linkNumber_j);
end