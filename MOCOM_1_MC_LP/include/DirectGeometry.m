%% DirectGeometry Function 
% Inputs: 
%   - qi : current links position;
%   - biTri: transformation matrix from link i to link i+1 for qi=0; 
%   - linkType: 0 for revolute, 1 for prismatic;
% output:
%   - biTei : transformation matrix from link i to link i+1 for the input qi.

function biTei = DirectGeometry(qi, biTri, linkType)
    % Revolute joint 
    if (linkType == 0) 
       %matrix of rotation around z-axis
       Rt(1,1) = cos(qi); Rt(1,2) = -sin(qi); Rt(1,3) = 0;  Rt(1,4) = 0;
       Rt(2,1) = sin(qi); Rt(2,2) = cos(qi);  Rt(2,3) = 0;  Rt(2,4) = 0;
       Rt(3,1) = 0;       Rt(3,2) = 0;        Rt(3,3) = 1;  Rt(3,4) = 0;
       Rt(4,1) = 0;       Rt(4,2) = 0;        Rt(4,3) = 0;  Rt(4,4) = 1;

   % post-multiplication of the transformation matrix by the matrix related
   % to the rotation
   biTei = biTri * Rt; 
   end

   % Prismatic joint
    if (linkType == 1) 
       %matrix of translation along z-axis
       Ts(1,1) = 1;       Ts(1,2) = 0;        Ts(1,3) = 0;  Ts(1,4) = 0;
       Ts(2,1) = 0;       Ts(2,2) = 1;        Ts(2,3) = 0;  Ts(2,4) = 0;
       Ts(3,1) = 0;       Ts(3,2) = 0;        Ts(3,3) = 1;  Ts(3,4) = qi;
       Ts(4,1) = 0;       Ts(4,2) = 0;        Ts(4,3) = 0;  Ts(4,4) = 1;

       %post-multiplication of the transformation matrix by the matrix related
       %to the translation
       biTei = biTri * Ts;
    end

end
