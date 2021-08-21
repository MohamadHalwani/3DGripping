%rosinit('192.168.50.73')


B_full = [  1   1   1   1   -1   -1   -1   -1;  
            1   1   -1   -1   1   1   -1   -1;
            1   -1   1   -1   1   -1   1   -1;
            1   1   1   1   1   1   1   1];


[transformation_publisher,transformation_msg] = rospublisher('/cluster_pose', 'geometry_msgs/Pose', "IsLatching", false);

sub = rossubscriber('/points1_cluster', "sensor_msgs/PointCloud");
%receive(sub,10); % Wait to receive first message

a = [];transformation_msg
b = [];

while(1)
    receive(sub)
    ros_cloud = sub.LatestMessage;
    if (length(ros_cloud) > 0)
        if length(ros_cloud.Points) > 1
            'in'
            target_cloud = convert_roscloud_to_affine_matrix(ros_cloud);
            [tranaformation_matrix, R, T, C, transformed_cloud] = solve_ICP_pertmutations(target_cloud, B_full);

            %convert trasnformation matrix to msg
            transformation_msg.Position.X = T(1)
            transformation_msg.Position.Y = T(2)
            transformation_msg.Position.Z = T(3)
            euler = rotm2eul(R, 'ZYX')
            quat = rotm2quat(R);
            transformation_msg.Orientation.W = euler(1)-3.1415926
            transformation_msg.Orientation.X = R(1,1)
            transformation_msg.Orientation.Y = quat(3)
            transformation_msg.Orientation.Z = R(2,2) 
            
             if isempty(a)
                 delete(a)
                 delete(b)
             end
             a = scatter3(transformed_cloud(1,:), transformed_cloud(2,:), transformed_cloud(3,:));
             hold on
             b = scatter3(target_cloud(1,:), target_cloud(2,:), target_cloud(3,:));
             
       
            transformation_publisher.send(transformation_msg);
        end
    else
        'out'
        pause(0.1)
    end
end



    

