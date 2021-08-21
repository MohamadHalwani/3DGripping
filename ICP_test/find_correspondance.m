function [source_correspond] = find_correspondance(target, source)
%FIND_CORRESPONDANCE Summary of this function goes here
%   Detailed explanation goes here
target_to_source_correspond = zeros(size(target,2),1);
distance_grid = zeros(size(target,2), size(source, 2));
for i=1:size(target,2)
    for j=1:size(source, 2)
        distance_grid(i,j) = sqrt( (target(1,i)-source(1,j))^2 + (target(2,i)-source(2,j))^2 + (target(3,i)-source(3,j))^2);
    end    
end


%METHOD A: shortest first
cropped_distance_grid = distance_grid;
for i=1:size(target,2)    
    [A_idx, B_idx] = find(distance_grid == min(cropped_distance_grid, [], 'all'));
    target_to_source_correspond(A_idx) = B_idx;
    [A_del_idx, B_del_idx] = find(cropped_distance_grid == min(cropped_distance_grid, [], 'all'));
    cropped_distance_grid(A_del_idx, :) = [];
    cropped_distance_grid(:, B_del_idx) = [];
end


%METHOD A: longest first first
% cropped_distance_grid = distance_grid;
% for i=1:size(target,2)    
%     row_min_distance_grid = min(cropped_distance_grid, [], 2);    
%     [A_idx, B_idx] = find(distance_grid == max(row_min_distance_grid));
%     target_to_source_correspond(A_idx) = B_idx;
%     [A_del_idx, B_del_idx] = find(cropped_distance_grid == max(row_min_distance_grid));
%     cropped_distance_grid(A_del_idx, :) = [];
%     cropped_distance_grid(:, B_del_idx) = [];
% end


source_correspond = source(:, target_to_source_correspond);

end

