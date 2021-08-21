%%
%define reference points 
B_full = [  1   1   1   1   0   0   0   0;  
            1   1   0   0   1   1   0   0;
            1   0   1   0   1   0   1   0;
            1   1   1   1   1   1   1   1];
 

B = B_full(:,[1, 3, 5, 7, 2, 6]);

%scaled parameters
A = [0.1    0.042    0.021    -0.023     0.1       0.02;
    -0.6    -0.63   -0.53   -0.58     -0.59    -0.53;  
    0.063    0.072    0.066    0.077    0.0         -0.01;
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

%alternatively 
A_inv = pinv(A')

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
%fitted_model = estimated_transformation_ab * estimated_B_scaled;
scatter3(fitted_model(1,:), fitted_model(2,:), fitted_model(3,:));

legend('model', 'measured', 'fitted')

%%
%unscaled transformation
% scaled_transformation_ba = [estimated_scale_x; estimated_scale_y; estimated_scale_z; 1] .* transformation_ba
% 
% transformation_ab = inv(scaled_transformation_ba)
% 
% %%
% %normalize in target frame
% estimated_transformation_ab_ = inv(transformation_ba)
% 
% unscaled_R_ab = transformation_ab(1:3,1:3);
% unscaled_T_ab = transformation_ab(1:3,4);
% 
% estimated_scale_x = 1/norm(unscaled_R_ab(1,:));
% estimated_scale_y = 1/norm(unscaled_R_ab(2,:));
% estimated_scale_z = 1/norm(unscaled_R_ab(3,:));
% 
% R_ab = [estimated_scale_x; estimated_scale_y; estimated_scale_z] .* unscaled_R_ab;
% T_ab = [estimated_scale_x; estimated_scale_y; estimated_scale_z] .* unscaled_T_ab;
% 
% estimated_transformation_ab = eye(4); 
% estimated_transformation_ab(1:3, 1:3) = R_ab;
% estimated_transformation_ab(1:3, 4) = T_ab;
% 
% 
% fitted_model = estimated_transformation_ab * estimated_B_scaled;
% scatter3(fitted_model(1,:), fitted_model(2,:), fitted_model(3,:));
% 
% legend('model', 'measured', 'fitted')

%%
%test SVD approach of https://ieeexplore-ieee-org.libconnect.ku.ac.ae/stamp/stamp.jsp?tp=&arnumber=88573
%this implementation assumes known scaling
scaled_x = 0.1;
scaled_y = 0.07;
scaled_z = 0.08;

B_scaled = B;
B_scaled(1,:) = scaled_x*B_scaled(1,:);
B_scaled(2,:) = scaled_y*B_scaled(2,:);
B_scaled(3,:) = scaled_z*B_scaled(3,:);

mu_A = sum(A(1:3,:), 2) / size(A,2);
mu_B = sum(B_scaled(1:3,:), 2) / size(B_scaled,2);

sigma2_A = norm(A(1:3,:) - mu_A)^2 / size(A,2);
sigma2_B = norm(B_scaled(1:3,:) - mu_B)^2 / size(B_scaled,2);

SIGMA_AB =  (A(1:3,:) - mu_A) * (B_scaled(1:3,:) - mu_B)' / size(A,2);

[U, sing, V] = svd(SIGMA_AB);

S = eye(size(U,2));
if det(SIGMA_AB) < 0
    S(end,end) = -1;
end

R_ab = U * S * V';
c = trace(sing*S) / (2*sigma2_B); 
T_ab = mu_A - c * R_ab * mu_B;

transformed_B = c * R_ab * diag([scaled_x, scaled_y, scaled_z]) * B_full(1:3,:) + T_ab;

scatter3(A(1,:),A(2,:),A(3,:))
hold on
scatter3(transformed_B(1,:),transformed_B(2,:),transformed_B(3,:))


