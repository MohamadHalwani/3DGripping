%%
%define reference points 
B = [1  -1  1   -1;
     -1 -1  1   1;
     1  1   1   1];

%scaled parameters
scale_x = 1.5;
scale_y = 0.5;
B_scaled = B;
B_scaled(1,:) = scale_x*B_scaled(1,:);
B_scaled(2,:) = scale_y*B_scaled(2,:);

%Define actual translation and rotation
theta = pi/4;
R_ab = [cos(theta)     -sin(theta);
    sin(theta)    cos(theta)];

T_ab = [2;  2];

transformation_ab = [R_ab  T_ab;  0 0 1];

A = transformation_ab * B_scaled;


%%
%Test SVD
[U, S, V] = svd(A');

S_zero = S==0;
s_man = S_zero + S;
s_inv =  (S~=0)./s_man;
A_inv = V * s_inv' * U';

transformation_ba = (A_inv * B')';

unscaled_R_ba = transformation_ba(1:2,1:2);
unscaled_T_ba = transformation_ba(1:2,3);

scaled_x = 1/norm(unscaled_R_ba(1,:));
scaled_y = 1/norm(unscaled_R_ba(2,:));

R_ba = [scaled_x; scaled_y] .* unscaled_R_ba
T_ba = [scaled_x; scaled_y] .* unscaled_T_ba
