clear all
close all
pkg load io

odom_file = '/home/li9i/catkin_ws/src/relief_devel/src/odom_tests_node/odom_test';

c = csv2cell(odom_file);

sec = cell2mat(c(2:end,1));
nsec = cell2mat(c(2:end,2));
x = cell2mat(c(2:end,3));
y = cell2mat(c(2:end,4));
z = cell2mat(c(2:end,5));
linear_x = cell2mat(c(2:end,6));
linear_y = cell2mat(c(2:end,7));
angular_x = cell2mat(c(2:end,8));
angular_y = cell2mat(c(2:end,9));
angular_z = cell2mat(c(2:end,10));

arr = [sec nsec x y z linear_x linear_y angular_x angular_y angular_z];

figure
hold on
%plot(x)
%plot(y)
% yaw
plot(z, 'b')
%plot(linear_x)
%plot(linear_y)
%plot(angular_x)
%plot(angular_y)
plot(angular_z, 'r')
