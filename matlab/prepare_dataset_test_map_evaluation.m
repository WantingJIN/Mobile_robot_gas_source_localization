close all; clear all; clc;
% for each test map, we need 2 datasets:
% 1: that contains the flow_def, the SDF, and an empty map for source pos
% 2: same as before, but one for each possible source position
% These datasets are used in the python code apply_DeepCFD, they are not
% for learning, they are for evaluation

% data files
flow_data_folder = '../openfoam/test_maps/';
flow_data_list = dir(flow_data_folder);
flow_data_list = flow_data_list([flow_data_list.isdir]);
flow_data_list = flow_data_list(~ismember({flow_data_list.name},{'.','..','0'}));

plume_data_folder = '../Results/test_maps/';
plume_data_list = dir(plume_data_folder);
plume_data_list = plume_data_list([plume_data_list.isdir]);
plume_data_list = plume_data_list(~ismember({plume_data_list.name},{'.','..','0'}));

% universal grid
x = round(2.75:0.1586:12.7418, 4);
y = round(0.01:0.0632:3.9916, 4);
[X,Y] = meshgrid(x,y);

counter = 0;

%% create 3D grid sets
for i=0:length(plume_data_list)-1
    
    % map number
    mapN = ['map' num2str(i)];
    
    % flow def
    flow_def = load([flow_data_folder mapN '/' mapN '_wind0.75/flow_def.mat']);

    % SDF
    SDF = load([flow_data_folder mapN '/' mapN '_wind0.75/SDF.mat']);
    
    %% dataset #1 (empty source position)
    
    % empty source position
    empty_source_pos = zeros(size(flow_def.flow_def));
    
    % putting everything together
%     ds1_x(1,1,:,:) = flow_def.flow_def;
%     ds1_x(1,2,:,:) = SDF.sdf;
%     ds1_x(1,3,:,:) = empty_source_pos;
    flow_def = flow_def.flow_def;
    sdf = SDF.sdf;
    source_pos = empty_source_pos;
    
    % save it
    %py_x = ds1_x;
    save([plume_data_folder mapN '/test_' mapN '_flowdef_sdf_sourcepos.mat'],'flow_def','sdf','source_pos', 'X', 'Y');
    %clear py_x;
    
    %% dataset #2 (one dataset for each possible source position)
    
%     counter = 0;
%     
%     % source position map
%     for sx = 2.75+0.25:0.5:12.75
%         for sy = 0+0.1:0.2:4
%             % distance from source
%             dist_to_source = zeros(size(flow_def.flow_def));
%             dist_to_source = sqrt((X-sx).^2 + (Y-sy).^2);
%             
%             % put everything together
%             counter = counter + 1;
%             ds2_x(counter,1,:,:) = flow_def.flow_def;
%             ds2_x(counter,2,:,:) = SDF.sdf;
%             ds2_x(counter,3,:,:) = 1./dist_to_source;
%         end
%     end
%     
%     % save it
%     py_x = ds2_x;
%     save([plume_data_folder mapN '/test_' mapN '_flowdef_sdf_allsourcepos.mat'], 'py_x', 'X', 'Y');
%     clear py_x;

end