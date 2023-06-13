clear all; close all; clc;

% Check the test map environments
general_folder = '/home/wjin/Workspace/continuous_STE/Results/test_maps';
map_number = 3;
file_name = [general_folder, '/map', num2str(map_number),'/test_map', num2str(map_number), '.environment'];

fileID = fopen(file_name,'r');

env_vertex = textscan(fileID,'%f %f','CommentStyle','//')

env_x = env_vertex{1};
env_y = env_vertex{2};

plot(env_x, env_y)
axis equal

hold on
obstacles_x=[];
obstacles_y=[];
obst_file = ['/home/wjin/Workspace/continuous_STE/openfoam/test_maps/map' num2str(map_number) '/map' num2str(map_number) '_wind1/' num2str(map_number) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
    obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
    obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end
patch(obstacles_x,obstacles_y ,[0.6 0.8 1], 'DisplayName','Obstacle','LineStyle', 'none');
set(gca,'Ytick',0:1:4)
grid on; axis equal; axis([2.25 13.25 0 4 0 2]);