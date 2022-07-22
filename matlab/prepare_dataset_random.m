close all; clear all; clc;

% data files
flow_data_folder = '../openfoam/training_random/training_random_wind1.0_angle0/';
flow_data_list = dir(flow_data_folder);
flow_data_list = flow_data_list([flow_data_list.isdir]);
flow_data_list = flow_data_list(~ismember({flow_data_list.name},{'.','..','0'}));

plume_data_folder = '../Results/training_random/';
plume_data_list = dir(plume_data_folder);
plume_data_list = plume_data_list([plume_data_list.isdir]);
plume_data_list = plume_data_list(~ismember({plume_data_list.name},{'.','..','0'}));

% universal grid
x = round(2.75:0.1586:12.7418, 4);
y = round(0.01:0.0632:3.9916, 4);
[X,Y] = meshgrid(x,y);

counter = 0;
data_size = 0;
 
% determine the size
for i=1:length(plume_data_list)
    concentration_plume_data_list = dir([plume_data_folder '/' plume_data_list(i).name]);
    concentration_plume_data_list = concentration_plume_data_list(~ismember({concentration_plume_data_list.name},{'.','..'}));
    data_size = data_size + length(concentration_plume_data_list);
end

py_x = zeros(data_size, 3, 64, 64);
py_y = zeros(data_size, 2, 64, 64);


%% create 3D grid sets
for i=1:length(plume_data_list)
    disp([num2str(i) '/' num2str(length(plume_data_list))]);
    
    % flow def
    flow_def = load([flow_data_folder plume_data_list(i).name '/flow_def.mat']);

    % SDF
    SDF = load([flow_data_folder plume_data_list(i).name '/SDF.mat']);
    
    % wind map
    wind = load([flow_data_folder plume_data_list(i).name '/wind_map.mat']);

    % source position and concentration
    concentration_plume_data_list = dir([plume_data_folder '/' plume_data_list(i).name]);
    concentration_plume_data_list = concentration_plume_data_list(~ismember({concentration_plume_data_list.name},{'.','..'}));
    
    for j=1:length(concentration_plume_data_list)
        disp(['               ' num2str(j) '/' num2str(length(concentration_plume_data_list))]);
        
        % read source position from the file name
        C = regexp(concentration_plume_data_list(j).name,'\d+\.?\d*','match');
        source_position_x = str2double(C{1}); 
        source_position_y = str2double(C{2});
        
        % distance from source
        dist_to_source = zeros(length(x), length(y));
        dist_to_source = sqrt((X-source_position_x).^2 + (Y-source_position_y).^2);
        
        % concentration avg and std
        c_avg = zeros(length(x), length(y));
        c_std = zeros(length(x), length(y));
        data = load([plume_data_folder  plume_data_list(i).name '/' concentration_plume_data_list(j).name]);
        if(size(data,1)~= length(x)*length(y)), continue; end
        for k=1:length(data(:,7))
            c_avg(y==data(k,8), x==data(k,7)) = data(k,10);
            c_std(y==data(k,8), x==data(k,7)) = data(k,11);
        end
        
        % put everything together
        counter = counter + 1;
        py_x(counter,1,:,:) = flow_def.flow_def;
        py_x(counter,2,:,:) = SDF.sdf;
        py_x(counter,3,:,:) = 1./dist_to_source;
%         py_x(counter,4,:,:) = wind.wind_x;
%         py_x(counter,5,:,:) = wind.wind_y;
        
        py_y(counter,1,:,:) = c_avg;
        py_y(counter,2,:,:) = c_std;
        
        % visualize to make sure we have all correct data
%         subplot(3,3,1);
%         imagesc(flow_def.flow_def);
%         title("flow def");
%         subplot(3,3,2);
%         imagesc(SDF.sdf);
%         title("SDF");
%         subplot(3,3,3);
%         imagesc(1./dist_to_source);
%         title("dist inverse to source");
%         subplot(3,3,4);
%         imagesc(wind.wind_x);
%         title("wind x");
%         subplot(3,3,5);
%         imagesc(wind.wind_y);
%         title("wind y");
%         subplot(3,3,7);
%         imagesc(c_avg);
%         title("C avg");
%         subplot(3,3,8);
%         imagesc(c_std);
%         title("C std");
%         pause()
    end

end

% randomize the order
rand_idx = randperm(size(py_x,1));
py_x = py_x(rand_idx, :,:,:);
py_y = py_y(rand_idx, :,:,:);

%% save 
save([plume_data_folder 'my_x_for_torch_200.mat'], 'py_x');
save([plume_data_folder 'my_y_for_torch_200.mat'], 'py_y');

test_x = py_x(1,:,:,:);
save([plume_data_folder 'input_stationary_channels_200.mat'], 'test_x', 'X', 'Y');