%% GetDirectGeometryFunction
%inputs: 
% - q : links current position ; 
% - biTri : vector of matrices containing the transformation matrices from
% link i to link i+1 for q=0. The size of biTri is (4,4,numberOfLinks)];
% - linkType: vector of size numberOfLinks identifying the joint type: 0 for revolute, 1 for
% prismatic.
% outputs:
% - biTei: vector of matrices containing the transformation matrices from link i to link i+1 for the input q. The
% size of biTei is equal to (4,4,numberOfLinks).

function [biTei] = GetDirectGeometry(q, biTri, linkType)

%initialization of the matrix
rows = size(biTri, 1);
columns = size(biTri, 2);
numberOfLinks = size (biTri, 3);
biTei = zeros( rows, columns, numberOfLinks );

%recursive filling of the biTei matrix with the output of DirectGeometry
for i = 1 : numberOfLinks
    
    biTei(:, :, i) = DirectGeometry( q(i), biTri(:, :, i), linkType(i));

end

end


