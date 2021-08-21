function [my_cloud] = convert_roscloud_to_affine_matrix(roscloud)
%CONVERT_ROSCLOUD_TO_AFFINE_MATRIX Summary of this function goes here
%   Detailed explanation goes here
my_cloud = zeros(4, length(roscloud.Points));

for i=1:length(roscloud.Points)
    my_cloud(:,i) = [roscloud.Points(i).X, roscloud.Points(i).Y, roscloud.Points(i).Z, 1]';
end

end

