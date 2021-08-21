%%
%define reference points 
B_full = [  1   1   1   1   -1   -1   -1   -1;  
            1   1   -1   -1   1   1   -1   -1;
            1   -1   1   -1   1   -1   1   -1;
            1   1   1   1   1   1   1   1];
        
 
%scaled parameters

%five points
A = [0.1    0.042    0.021    -0.023     0.1       0.02;
    -0.6    -0.63   -0.53   -0.58     -0.59    -0.53;  
    0.063    0.072    0.066    0.077    0.0         -0.01;
    1       1       1       1       1           1];


%four points
A = [0.1    0.042     0.1       0.02;
    -0.6    -0.63     -0.59    -0.53;  
    0.063    0.072   0.0         -0.01;
    1       1       1   1];



%three points
% A = [0.1     0.1       0.02;
%     -0.6     -0.59    -0.53;  
%     0.063   0.0         -0.01;
%     1    1   1]

% %%
% figure()
% %scatter3(B_full(1,:), B_full(2,:), B_full(3,:));
% scatter3(A(1,:), A(2,:), A(3,:));
% hold on
% transformation_matrix_full = eye(4);
% B_transformed = B_full;
% for i=1:10
%     %find correspondance 
%     B_correspond = find_correspondance(A, B_transformed);
%     
%     %solve ICP
%     [transformation_matrix, ~, ~, ~] = solve_transform(A, B_correspond);
%     transformation_matrix_full = transformation_matrix * transformation_matrix_full;
%     
%     %plot iteration
%     B_transformed = transformation_matrix * B_transformed;
%     
%     g = scatter3(B_transformed(1,:), B_transformed(2,:), B_transformed(3,:));
%     pause(2)
%     delete(g)
% end
% 
% g = scatter3(B_transformed(1,:), B_transformed(2,:), B_transformed(3,:));
%     
% %%
% %plot correspondance
% B_correspond = find_correspondance(A, B_transformed);
% scatter3(B_transformed(1,:), B_transformed(2,:), B_transformed(3,:)) 
% hold on
% scatter3(A(1,:), A(2,:), A(3,:))
% 
% for i=1:size(A,2)
%     plot3([A(1,i), B_correspond(1,i)], [A(2,i), B_correspond(2,i)], [A(3,i), B_correspond(3,i)])
% end

%%
%iterate all possible correspondance and find best fit
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
        [transformation_matrix, ~, ~, ~] = solve_transform(A, B_correspond);
        error = norm(transformation_matrix * B_correspond - A, 'fro');
        combination_matrix = [combination_matrix; correspondance_vector'];
        error_vector = [error_vector; error];
    end
end

%%
%solve for best error
best_crr_idx = min(find(error_vector == min(error_vector)));
combination = combination_matrix(best_crr_idx, :);
B_correspond = B_full(:, combination);        
[transformation_matrix, R, T, C] = solve_transform(A, B_correspond);

B_transformed = transformation_matrix * B_full;
g = scatter3(B_transformed(1,:), B_transformed(2,:), B_transformed(3,:));
hold on
g = scatter3(A(1,:), A(2,:), A(3,:));