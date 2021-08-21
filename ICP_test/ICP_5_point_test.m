%%
%define reference points 
B_full = [1  -1  1   -1  1   -1  1   -1;
         -1 -1  1   1   -1  -1  1   1;
         -1 -1  -1  -1  1   1   1   1;
          1   1   1   1  1   1   1   1];
 

B = B_full(:,[1, 3, 2, 4, 5, 6]);

%scaled parameters
A = [0.1    0.04    0.02    -0.02     0.1       0.02;
    -0.6    -0.63   -0.53   -0.58     -0.59    -0.53;  
    0.06    0.07    0.07    0.08    0.0         -0.01;
    1       1       1       1       1           1];

%%
%scatter points
scatter3(B(1,:), B(2,:), B(3,:));
hold on
scatter3(A(1,:), A(2,:), A(3,:));
%%
%Test SVD
[U, S, V] = svd(A');

S_zero = S==0;
s_man = S_zero + S;
s_inv =  (S~=0)./s_man;
A_inv = V * s_inv' * U';

%alternatively A_inv = pinv(A)

transformation_ba = (A_inv * B')';

unscaled_R_ba = transformation_ba(1:3,1:3);
unscaled_T_ba = transformation_ba(1:3,4);

estimated_scale_x = 1/norm(unscaled_R_ba(1,:));
estimated_scale_y = 1/norm(unscaled_R_ba(2,:));
estimated_scale_z = 1/norm(unscaled_R_ba(3,:));

R_ba = [estimated_scale_x; estimated_scale_y; estimated_scale_z] .* unscaled_R_ba
T_ba = [estimated_scale_x; estimated_scale_y; estimated_scale_z] .* unscaled_T_ba

%%
%plot points after transformation 
figure()
scatter3(B(1,:), B(2,:), B(3,:));
hold on
scatter3(A(1,:), A(2,:), A(3,:));

%transofomed B
estimated_R_ab = transpose(R_ba);
estimated_T_ab = -estimated_R_ab * T_ba;

estimated_B_scaled = B_full;
estimated_B_scaled(1,:) = estimated_scale_x*estimated_B_scaled(1,:);
estimated_B_scaled(2,:) = estimated_scale_y*estimated_B_scaled(2,:);
estimated_B_scaled(3,:) = estimated_scale_z*estimated_B_scaled(3,:);
%%
estimated_transformation_ab = [estimated_R_ab, estimated_T_ab; 0 0 0 1];
fitted_model = inv(transformation_ba) * B_full;
%fitted_model = inv(transformation_ba) * B_full;
fitted_model = estimated_transformation_ab * estimated_B_scaled;
scatter3(fitted_model(1,:), fitted_model(2,:), fitted_model(3,:));

legend('model', 'measured', 'fitted')



