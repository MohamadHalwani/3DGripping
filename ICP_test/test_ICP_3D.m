%%
%define reference points 
B = [1  -1  1   -1  1   -1  1   -1;
     -1 -1  1   1   -1  -1  1   1;
     -1 -1  -1  -1  1   1   1   1;
     1   1   1   1  1   1   1   1];

%scaled parameters
scale_x = 1.5;
scale_y = 0.5;
scale_z = 1;
B_scaled = B;
B_scaled(1,:) = scale_x*B_scaled(1,:);
B_scaled(2,:) = scale_y*B_scaled(2,:);
B_scaled(3,:) = scale_z*B_scaled(3,:);

%Define actual translation and rotation
theta = pi/4;
R_ab = eul2rotm([theta 0 0])

T_ab = [2;  2; 2];

transformation_ab = [R_ab  T_ab;  0 0 0 1];

A = transformation_ab * B_scaled;

%%
%add noise to A
noise_magnitude = 0.3;
A_noise = A;
for i=1:size(A,2)
    %random unit vector
    rand_unit = rand(3,1);
    rand_unit = rand_unit / norm(rand_unit);
    A_noise(1:3, i) = A_noise(1:3, i) + rand * noise_magnitude * rand_unit;
end
%%
%scatter points
scatter3(B(1,:), B(2,:), B(3,:));
hold on
scatter3(A(1,:), A(2,:), A(3,:));
scatter3(A_noise(1,:), A_noise(2,:), A_noise(3,:));

%%
%Test SVD
[U, S, V] = svd(A_noise');

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
scatter3(A_noise(1,:), A_noise(2,:), A_noise(3,:));

%transofomed B
estimated_R_ab = transpose(R_ba);
estimated_T_ab = -estimated_R_ab * T_ba;

estimated_B_scaled = B;
estimated_B_scaled(1,:) = estimated_scale_x*estimated_B_scaled(1,:);
estimated_B_scaled(2,:) = estimated_scale_y*estimated_B_scaled(2,:);
estimated_B_scaled(3,:) = estimated_scale_z*estimated_B_scaled(3,:);

estimated_transformation_ab = [estimated_R_ab, estimated_T_ab; 0 0 0 1];
fitted_model = estimated_transformation_ab * estimated_B_scaled;
scatter3(fitted_model(1,:), fitted_model(2,:), fitted_model(3,:));

legend('model', 'measured', 'fitted')



