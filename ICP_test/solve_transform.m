function [transformation_matrix, R, T, C] = solve_ICP(target, source)
%SOLVE_ICP Summary of this function goes here
%   SVD approach of https://ieeexplore-ieee-org.libconnect.ku.ac.ae/stamp/stamp.jsp?tp=&arnumber=88573

scaled_x = 0.1;
scaled_y = 0.07;
scaled_z = 0.08;

A = target; 
B = source;

mu_A = sum(A(1:3,:), 2) / size(A,2);
mu_B = sum(B(1:3,:), 2) / size(B,2);

sigma2_A = norm(A(1:3,:) - mu_A, 'fro')^2 / size(A,2);
sigma2_B = norm(B(1:3,:) - mu_B, 'fro')^2 / size(B,2);

SIGMA_AB =  (A(1:3,:) - mu_A) * (B(1:3,:) - mu_B)' / size(A,2);

[U, sing, V] = svd(SIGMA_AB);

S = eye(size(U,2));
if det(SIGMA_AB) < 0
    S(end,end) = -1;
end

R_ab = U * S * V';
c = trace(sing*S) / (sigma2_B); 
T_ab = mu_A - c * R_ab * mu_B;

transformation_matrix = eye(4);
transformation_matrix(1:3, 1:3) = c * R_ab;
transformation_matrix(1:3,4) = T_ab;

R = R_ab;
T = T_ab;
C = c;

end

