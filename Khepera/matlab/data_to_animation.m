close all; clear all; clc;

% the general folder
general_folder = 'C:\Users\rahbar\Desktop\ICRA2020_video\log_STE_multi_v10';
folder_list = dir(general_folder);
folder_list = folder_list(~ismember({folder_list.name},{'.','..','animation'}));
folder_list = folder_list([folder_list.isdir]);
robot_number = length(folder_list);

% parameters
X = linspace(2.5,12.5,40);
Y = linspace(0,4,20);
Q = linspace(1,800,10);
A = linspace(0.01,0.2,10); %0.01
B = linspace(0.1,1,10); %0.1

[Xg,Yg] = meshgrid(X,Y);

% error std
sigmaD = 30;
sigmaT = 125;

source_real_position = [12.5,2,0];
Q_min = 1; Q_max = 800;
source_position_min = [2.5,0,0];
source_position_max = [12.5,4,2];
a_y_min = 0.01; a_y_max = 0.2;
b_y_min = 0.1; b_y_max = 1;

like_sum = zeros(1,length(X)*length(Y)*length(Q)*length(A)*length(B));

anim_folder = [general_folder,'/animation'];
mkdir(anim_folder);

% set(groot,'defaultFigurePaperPositionMode','manual');
f = figure('Position',[601.0000  111.4000  397.6000  650.6000]);

% first frame:
post_map = ones(20,40); post_map = post_map./sum(post_map,[1,2]);
scatter(Yg(:),Xg(:),30,post_map(:),'s','filled');
axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
title(['Posterior pdf on X and Y \newline  ']);
xlabel('Y(m)'); ylabel('X(m)');
load MyColormap; colormap(mymap); colorbar;
ax = gca; ax.XDir = 'reverse';
frames(1) = getframe(f);
saveas(f, [anim_folder,'/0.png']);

for k=1:robot_number
    folder_name = [folder_list(k).folder, '/', folder_list(k).name];
    % data
    if(exist([folder_name '/position_history.csv']))
        position_history = dlmread([folder_name '/position_history.csv']);
        measurement_history = dlmread([folder_name '/measurement_history.csv']);
    end
    p{k} = dlmread([folder_name '/goal_position.csv']);
%     entropy = dlmread([folder_name '/entropy.csv']);
end

for k=1:robot_number
    ID(:,k) = ismember(position_history(:,2:3),p{k}(:,2:3),'rows');
end

    %% calculate posterior without MCMC
    for i=1:length(measurement_history)
        % robots position
        for k=1:robot_number
            if(ID(i,k) == 1 || i==1)
                xr(i,k) = position_history(i, 2);
                yr(i,k) = position_history(i, 3);
            else
                xr(i,k) = xr(i-1,k);
                yr(i,k) = yr(i-1,k);
            end
        end
        xr_h = position_history(i, 2);
        yr_h = position_history(i, 3);
        % robot's measured concentration
        cr = measurement_history(i, 2);
        m=1;
        for q=Q
            for x=X
                for y=Y
                    for sig_a=A
                        for sig_b=B
                            % model
                            adjusted_x = (x-xr_h);
                            adjusted_y = (y-yr_h);
                            %rotation
                            theta = 5/180*pi;
                            rotated_x = adjusted_y*sin(theta) + adjusted_x*cos(theta);
                            rotated_y = adjusted_y*cos(theta) - adjusted_x*sin(theta);
                            adjusted_x = rotated_x;
                            adjusted_y = rotated_y;
                            if(adjusted_x < 0)
                                model_value(m) = 0;
                            else
                                sigma_y = sig_a * adjusted_x + sig_b;
                                sigma_z = 1; z=0; zr=0;
                                model_value(m) = (q/(2*pi*1.0*sigma_y*sigma_z)) * exp( -((adjusted_y*adjusted_y)/(2*sigma_y*sigma_y)) - (((z-zr)*(z-zr))/(2*sigma_z*sigma_z)));
                            end
                            %likelihood
                            likelihood(m) = exp(-0.5 * ((cr - model_value(m)).^2) / (sigmaD*sigmaD + sigmaT*sigmaT));
                            like_sum(m) = like_sum(m) + ((cr - model_value(m)).^2) / (sigmaD*sigmaD + sigmaT*sigmaT);
                            parameter_combination(:,m) = [q;x;y;sig_a;sig_b];
                            m = m+1;
                        end
                    end
                end
            end
        end
        %posterior
        posterior = exp(-0.5 .* like_sum);
        posterior = posterior./sum(posterior);
        %marginal on xy
        post_map = permute(sum(reshape(posterior,length(B),length(A),length(Y),length(X),length(Q)),[1,2,5]),[4,3,1,2])';
        scatter(Yg(:),Xg(:),30,post_map(:),'s','filled');
        hold on;
        for k=1:robot_number
            plot(yr(i,k),xr(i,k),'ko','MarkerSize',7,'MarkerFaceColor','g');
            plot(yr(i,k),xr(i,k),'w*','MarkerSize',7,'MarkerFaceColor','w');
        end
        axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
        title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
        xlabel('Y(m)'); ylabel('X(m)');
        load MyColormap; colormap(mymap); colorbar;
        ax = gca; ax.XDir = 'reverse';
        hold off;
        pause(0.2);
        %frames(i+1) = getframe(f);
        saveas(f, [anim_folder,'/',num2str(i),'.png']);
    end
    % time
    %time(1) = 0;
    %time(2:length(entropy)+1) = ((entropy(:,2)-entropy(1,2))./1000)+12;

%clearvars -except time frames general_folder


% counter = 1;
% for i=1:(time(end)+5)*25/8
%     if(i >= time(counter+1)*25/8)
%         counter = counter+1;
%     end
%     frames_30(i) = frames(counter);
% end
% 
% %movie making
% v = VideoWriter([general_folder,'.mp4'],'MPEG-4');
% v.Quality = 75;
% open(v);
% writeVideo(v,frames_30);
% close(v);
