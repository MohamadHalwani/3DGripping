function [transformation_matrix, R, T, C, transformed_cloud] = solve_ICP_pertmutations(target,source)
%SOLVE_ICP_PERTMUTATIONS Summary of this function goes here
%   Detailed explanation goes here

A = target;
B_full = source;

i=1;
iterator = 1;
error_vector = [];
combination_matrix = [];
correspondance_vector = ones(size(A, 2), 1);
while correspondance_vector(1) == 1
    correspondance_vector(end) = correspondance_vector(end) + 1;
    while max(correspondance_vector) > size(B_full, 2)
        idx = min(find(correspondance_vector > size(B_full, 2)));
        correspondance_vector(idx) = 1;
        correspondance_vector(idx-1) = correspondance_vector(idx-1) + 1;            
    end      
    if (length(correspondance_vector) == length(unique(correspondance_vector)))
        B_correspond = B_full(:, correspondance_vector);        
        [temp_transformation_matrix, ~, ~, ~] = solve_transform(A, B_correspond);
        error = norm(temp_transformation_matrix * B_correspond - A, 'fro');
        combination_matrix = [combination_matrix; correspondance_vector'];
        error_vector = [error_vector; error];
    end
end

best_crr_idx = min(find(error_vector == min(error_vector)));
combination = combination_matrix(best_crr_idx, :);
B_correspond = B_full(:, combination);        
[transformation_matrix, R, T, C] = solve_transform(A, B_correspond);

transformed_cloud = transformation_matrix * B_full;

end

