% === Base experiment path (we assume only one subfolder) ===
base_path = '/home/wjin/Workspace/Multi-robot_GSL/Results/Multi-GSL/REGION_GROWING_D/9';
d = dir(base_path);
subfolder = d([d.isdir] & ~startsWith({d.name}, '.'));
if isempty(subfolder)
    error('No subfolder found inside cluster_analysis');
end
exp_folder = fullfile(base_path, subfolder(1).name);

% === Visualization speed settings ===
show_cell_indices = false;
repr_vel = 2;
video_repr_vel = repr_vel*10;

% === Robot IDs and colors ===
robot_ids = [101, 102, 103];
robot_colors = {[1 0 0], [0 0 1], [0 0.6 0]};

% === Load data ===
positions = cell(1,3);
goal_positions = cell(1,3);
reached_goals = zeros(1,3);
goal_update_count = 0;

for r = 1:3
    pos_path = fullfile(exp_folder, num2str(robot_ids(r)), 'whole_position_history.csv');
    goal_path = fullfile(exp_folder, num2str(robot_ids(r)), 'goal_position.csv');
    positions{r} = round(readmatrix(pos_path)(:,1:2), 1);
    goal_positions{r} = round(readmatrix(goal_path)(:,2:3), 1);
end

source_path = fullfile(exp_folder, '101','source_pos.csv');
tokens = sscanf(fileread(source_path), 'source_position %f %f %f');
source_x = tokens(1); source_y = tokens(2);

intensity_data = readmatrix(fullfile(exp_folder, '101', 'posterior_pdf_3D.csv'));
belief_data = readmatrix(fullfile(exp_folder, '101', 'posterior_pdf_3D_cluster.csv'));
belief_x = round(belief_data(:,1), 1); belief_y = round(belief_data(:,2), 1);
belief_maps = belief_data(:, 3:end);

% === Figure and patches ===
figure('Position', [100, 100, 1200, 400]);
axis equal; hold on;
xlabel('X (m)'); ylabel('Y (m)');
title('FSM cycle: 1');
set(gcf, 'KeyPressFcn', @keypress_callback);
global is_paused; is_paused = false;

cols = 20; rows = 20;
x_edges = linspace(3, 12.5, cols+1);
y_edges = linspace(0.16, 3.8, rows+1);
cell_widths = diff(x_edges);
cell_heights = diff(y_edges);
cell_patches = gobjects(rows, cols);

for i = 1:cols
    for j = 1:rows
        x = x_edges(i); y = y_edges(j);
        xv = [x, x+cell_widths(i), x+cell_widths(i), x];
        yv = [y, y, y+cell_heights(j), y+cell_heights(j)];
        cell_patches(j,i) = patch(xv, yv, [1 1 1], 'EdgeColor', 'none');
        if show_cell_indices
            cx = x + cell_widths(i)/2;
            cy = y + cell_heights(j)/2;
            text(cx, cy, num2str((j-1)*cols + i), 'FontSize', 8, 'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
        end
    end
end

% === Obstacles, walls, source marker ===
wall_thickness = 0.1;
rectangle('Position', [-wall_thickness, -wall_thickness, 16 + wall_thickness, wall_thickness], 'FaceColor', [0.3 0.3 0.3]);
rectangle('Position', [0, 4, 16 + wall_thickness, wall_thickness], 'FaceColor', [0.3 0.3 0.3]);
rectangle('Position', [-wall_thickness, 0, wall_thickness, 4 + wall_thickness], 'FaceColor', [0.3 0.3 0.3]);
rectangle('Position', [16, 0 - wall_thickness, wall_thickness, 4 + wall_thickness], 'FaceColor', [0.3 0.3 0.3]);
rectangle('Position', [8.0, 0.0, 0.5, 2.0], 'FaceColor', [0.3 0.3 0.3]);
rectangle('Position', [4.5, 2.0, 0.5, 2.0], 'FaceColor', [0.3 0.3 0.3]);
plot(source_x, source_y, 'p', 'MarkerSize', 16, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');

% === Robot & goal markers ===
robot_markers = gobjects(1,3);
goal_markers = gobjects(1,3);
for r = 1:3
    robot_markers(r) = plot(NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', robot_colors{r}, 'MarkerEdgeColor', 'k');
    goal_markers(r) = plot(NaN, NaN, 's', 'MarkerSize', 8, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
end

v = VideoWriter(fullfile(base_path, 'video_output.avi'), 'Motion JPEG AVI');
v.FrameRate = video_repr_vel;
open(v);

max_steps = max(cellfun(@(p) size(p,1), positions));
goal_idx = ones(1, 3);
waiting = false(1,3);
last_pos = zeros(3, 2);
t_idx = ones(1, 3);
done = false(1, 3);

for frame_t = 1:10000
    if all(reached_goals)
        for r = 1:3
            goal_idx(r) = goal_idx(r) + 1;
            waiting(r) = false;
        end
        reached_goals = zeros(1,3);
        goal_update_count = goal_update_count + 1;

        if goal_update_count <= size(intensity_data, 1)
            idx = goal_update_count;
            intensity_map = intensity_data(idx, :);
            intensity_map = (intensity_map - min(intensity_map)) / max(eps, max(intensity_map) - min(intensity_map));

            cmap = parula(256);
            for col = 1:cols
                for row = 1:rows
                    map_idx = (row - 1) * cols + col;
                    intensity = max(0, min(1, intensity_map(map_idx)));
                    set(cell_patches(row, col), 'FaceColor', cmap(round(intensity*255)+1, :));
                end
            end
            colormap(parula);
            caxis([0 1]);
            colorbar;
            title(sprintf('FSM cycle: %d', idx), 'FontSize', 14, 'FontWeight', 'bold');
        end
    end

    for r = 1:3
        if ~done(r) && t_idx(r) <= size(positions{r}, 1)
            curr_pos = positions{r}(t_idx(r), :);
            if goal_idx(r) <= size(goal_positions{r}, 1)
                gx = goal_positions{r}(goal_idx(r), 1);
                gy = goal_positions{r}(goal_idx(r), 2);
                dist = norm(curr_pos - [gx, gy]);
                reached_goals(r) = dist < 0.15;
                set(goal_markers(r), 'XData', gx, 'YData', gy);
            end

            if ~reached_goals(r)
                set(robot_markers(r), 'XData', curr_pos(1), 'YData', curr_pos(2));
                last_pos(r, :) = curr_pos;
                t_idx(r) = t_idx(r) + repr_vel;
                if t_idx(r) > size(positions{r}, 1)
                    t_idx(r) = size(positions{r}, 1);
                    done(r) = true;
                end
            else
                set(robot_markers(r), 'XData', last_pos(r, 1), 'YData', last_pos(r, 2));
            end
        else
            done(r) = true;
        end
    end

    drawnow;
    while is_paused
        pause(0.1);
    end
    pause(0.00001);
    frame = getframe(gcf);
    writeVideo(v, frame);

    if all(done)
        expected_data = readmatrix(fullfile(exp_folder, '101', 'expected_a_posteriori.csv'));
        estimated_x = expected_data(end, 2);
        estimated_y = expected_data(end, 3);
        plot(estimated_x, estimated_y, 'p', 'MarkerSize', 16, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1, 0.75, 0.1]);

        x_error = abs(estimated_x - source_x);
        y_error = abs(estimated_y - source_y);
        if x_error < 1 && y_error < 0.5
            msg = 'SOURCE FOUND ✅'; color = [0.1 0.6 0.1];
        else
            msg = 'SOURCE NOT FOUND ❌'; color = [0.8 0.1 0.1];
        end
        text(9.5, -1, msg, 'FontSize', 14, 'FontWeight', 'bold', 'Color', color, 'Units', 'data');

        for i=1:20
            writeVideo(v, getframe(gcf));
        end
        close(v);
        break;
    end
end

function keypress_callback(~, event)
    if strcmp(event.Key, 'space')
        global is_paused;
        is_paused = ~is_paused;
        disp(is_paused * '⏸️  Paused. Press [space] to resume.' + ~is_paused * '▶️  Resumed.');
    end
end