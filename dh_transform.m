function [ T ] = dh_transform( theta, d, a, alpha )
%dh_transform Summary of this function goes here
%   Detailed explanation goes here
    
i = [1; 0; 0];
k = [0; 0; 1];

ix = [0 0 0;
      0 0 -1;
      0 1 0];
  
kx = [0 -1 0;
      1 0 0;
      0 0 0];
 
angle_rot = expm(theta*kx);
angle_trans = [ angle_rot zeros(3,1);
                zeros(1,3) 1];

offset_trans = [ eye(3) d*k;
                 zeros(1,3) 1];

length_trans = [ eye(3) a*i;
                 zeros(1,3) 1];

twist_rot = expm(alpha*ix);
twist_trans = [ twist_rot zeros(3,1);
                zeros(1,3) 1];

T = angle_trans * offset_trans * length_trans * twist_trans;

end

