%% GetDirectGeometryFunction
% inputs: 
%   - q : links current position ; 
%   - biTri : vector of matrices containing the transformation matrices from
%             link i to link i+1 for q=0. 
%             The size of biTri is (4,4,linkNumber)];
%   - linkType: vector of size numberOfLinks identifying the joint type: 0 for revolute, 1 for
%               prismatic.
% outputs:
%   - biTei: vector of matrices containing the transformation matrices 
%            from link i to link i+1 for the input q. The size of biTei is equal 
%            to (4,4, linkNumber).

function [biTei] = GetDirectGeometry(q, biTri, linkType)

%initialization of the matrix
rows = size(biTri, 1);
columns = size(biTri, 2);
linkNumber = size (biTri, 3);
biTei = zeros( rows, columns, linkNumber );

%recursive filling of the biTei matrix with the output of DirectGeometry
for i = 1 : linkNumber
    
    biTei(:, :, i) = DirectGeometry( q(i), biTri(:, :, i), linkType(i));

end

end


