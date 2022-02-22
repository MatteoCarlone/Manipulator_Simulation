%% VersorLemma function
% inputs:
% - r1: the first rotation matrix
% - r2: the second rotation matrix
% output:
% - c: the Vect3 representing the axis around which r1 should rotate to reach r2, where its modulus is the angle

function c = VersorLemma(r1, r2)
threshold = 1 * 10^(-4);

%Column vectors of the rotation matrices  
ia = r1(:, 1);
ib = r2(:, 1);
ja = r1(:, 2);
jb = r2(:, 2);
ka = r1(:, 3);
kb = r2(:, 3);

%Computing vector lemma for frames: geometric form
vsinth = ( cross(ia, ib) + cross(ja, jb) + cross(ka, kb) ) / 2 ;
sinth = norm( vsinth );
costh = ( dot(ia, ib) + dot(ja, jb) + dot(ka, kb) - 1 ) / 2;

if(costh >= (1 - threshold)) %Case theta ~0
   v = [0, 0, 0]';
   theta = 0;
elseif(abs(costh) < 1 - threshold) %Case 0 < theta < pi
   theta = atan2(sinth, costh);
   v = vsinth / sinth;
else %Case theta ~ pi
   theta = pi;
   v = [0, 0, 0]';
   R = r1(:,:) + r2(:,:);
   if(norm(R(:, 1) ) ~= 0)
       v = R(:, 1);
   end
   if(norm(R(:, 2)) > norm(v) )
       v = R(:, 2);
   end
   if(norm( R(:, 3) ) > norm(v))
      v = R(:, 3); 
   end    
end
    
%Computing axis and magnitude around which r1 should rotate to reach r2
c = v * theta;
end

