% === Base experiment path (we assume only one subfolder) ===
base_path = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/cluster_analysis';
d = dir(base_path);
subfolder = d([d.isdir] & ~startsWith({d.name}, '.'));  % filter out hidden/system folders

if isempty(subfolder)
    error('No subfolder found inside cluster_analysis');
end

exp_folder = fullfile(base_path, subfolder(1).name);  % full path to experiment folder

% === Visualization speed settings ===
repr_vel = 5;           % How many steps to skip per frame (for robot position update)
video_repr_vel = 20;    % Frame rate of the output video

% === Robot IDs and corresponding colors ===
robot_ids = [101, 102, 103];
robot_colors = {[1 0 0], [0 0 1], [0 0.6 0]}; % red, blue, green

% === Load position history for each robot ===
positions = cell(1,3);
for r = 1:3
    pos_path = fullfile(exp_folder, num2str(robot_ids(r)), 'whole_position_history.csv');
    pos_data = readmatrix(pos_path);
    positions{r} = round(pos_data(:, 1:2), 1);  % round X, Y positions to 1 decimal
end

% === Load belief map (only from robot 101) ===
belief_path = fullfile(exp_folder, '101', 'posterior_pdf_3D_cluster.csv');
belief_data = readmatrix(belief_path);
belief_x = round(belief_data(:, 1), 1);
belief_y = round(belief_data(:, 2), 1);
belief_maps = belief_data(:, 3:end);  % columns containing map values

% === Load goal positions for each robot ===
goal_positions = cell(1,3);
goal_idx = ones(1,3);  % keep track of which goal each robot is targeting
for r = 1:3
    goal_path = fullfile(exp_folder, num2str(robot_ids(r)), 'goal_position.csv');
    goal_data = readmatrix(goal_path);
    goal_positions{r} = round(goal_data(:, 2:3), 1);  % X, Y columns
end

% === Load ground truth gas source position ===
source_path = fullfile(exp_folder, '101','source_pos.csv');
source_text = fileread(source_path);
tokens = sscanf(source_text, 'source_position %f %f %f');
source_x = tokens(1);
source_y = tokens(2);

% === Setup figure ===
figure('Position', [100, 100, 1200, 400]);
axis equal;
hold on;
wall_thickness = 0.1;
axis([-wall_thickness 16 + wall_thickness -wall_thickness 4 + wall_thickness]);
xlabel('X (m)');
ylabel('Y (m)');
title('Animated Belief Maps');

% === Cell grid setup (20x20 grid) ===
cols = 20; rows = 20;
cell_width = 16 / cols;
cell_height = 4 / rows;

% === Define soft colors for class coloring ===
green_soft = [0.75 0.95 0.75];
blue_soft  = [0.75 0.75 0.95];
red_soft   = [0.95 0.75 0.75];
yellow = [1, 0.75, 0.1];  % used for estimated source
dark_gray  = [0.3 0.3 0.3];  % used for walls and obstacles

% === Draw grid cells ===
cell_handles = gobjects(rows, cols);
for i = 1:cols
    for j = 1:rows
        x = (i - 1) * cell_width;
        y = (j - 1) * cell_height;
        cell_handles(j, i) = rectangle('Position', [x, y, cell_width, cell_height], ...
            'FaceColor', [1 1 1], 'EdgeColor', [0.85 0.85 0.85]);
    end
end

% === Draw walls ===
rectangle('Position', [-wall_thickness, -wall_thickness, 16 + wall_thickness, wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [0, 4, 16 + wall_thickness, wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [-wall_thickness, 0, wall_thickness, 4 + wall_thickness], 'FaceColor', dark_gray);
rectangle('Position', [16, 0 - wall_thickness, wall_thickness, 4 + wall_thickness], 'FaceColor', dark_gray);

% === Draw obstacles ===
rectangle('Position', [8.0, 0.0, 0.5, 2.0], 'FaceColor', dark_gray);
rectangle('Position', [4.5, 2.0, 0.5, 2.0], 'FaceColor', dark_gray);

% === Plot ground truth gas source (black star) ===
plot(source_x, source_y, 'p', 'MarkerSize', 16, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');

% === Initialize robot and goal markers ===
robot_markers = gobjects(1,3);
goal_markers = gobjects(1,3);
for r = 1:3
    robot_markers(r) = plot(NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', robot_colors{r}, 'MarkerEdgeColor', 'k');
    goal_markers(r) = plot(NaN, NaN, 'x', 'MarkerSize', 10, ...
        'LineWidth', 2, 'Color', robot_colors{r});
end

% === Create video writer object ===
output_video_path = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/videos/animation.avi';
v = VideoWriter(output_video_path, 'Motion JPEG AVI');
v.FrameRate = video_repr_vel;
open(v);

% === Prepare iteration variables ===
max_steps = max(cellfun(@(p) size(p,1), positions));
goal_idx = ones(1, 3);
waiting = false(1,3);
last_pos = zeros(3, 2);
t_idx = ones(1, 3);
done = false(1, 3);

% === Animation loop ===
for frame_t = 1:10000  % large upper bound to avoid infinite loops
    all_reached = true;

    for r = 1:3
        if ~done(r) && t_idx(r) <= size(positions{r}, 1)
            curr_pos = positions{r}(t_idx(r), :);

            % Check if robot has reached its current goal
            if goal_idx(r) <= size(goal_positions{r}, 1)
                gx = goal_positions{r}(goal_idx(r), 1);
                gy = goal_positions{r}(goal_idx(r), 2);

                if isequal(round(curr_pos,1), [gx, gy])
                    waiting(r) = true;
                else
                    waiting(r) = false;
                    all_reached = false;
                end
            end

            % Update robot marker position
            if ~waiting(r)
                set(robot_markers(r), 'XData', curr_pos(1), 'YData', curr_pos(2));
                last_pos(r, :) = curr_pos;
                t_idx(r) = t_idx(r) + repr_vel;
                if t_idx(r) > size(positions{r}, 1)
                    t_idx(r) = size(positions{r}, 1);
                    done(r) = true;
                end
            else
                % Stay at last known position
                set(robot_markers(r), 'XData', last_pos(r,1), 'YData', last_pos(r,2));
            end

            % Update goal marker
            if goal_idx(r) <= size(goal_positions{r}, 1)
                set(goal_markers(r), 'XData', gx, 'YData', gy);
            end
        else
            done(r) = true;
        end
    end

    % If all robots have reached their goals, update to next set
    if all_reached
        for r = 1:3
            goal_idx(r) = goal_idx(r) + 1;
            waiting(r) = false;
        end
    end

    % === Update belief map (only when robot 101 moves) ===
    rx = last_pos(1,1);
    ry = last_pos(1,2);
    idx = find(belief_x == rx & belief_y == ry, 1);
    if ~isempty(idx)
        delete(findall(gca, 'Type', 'text'));  % clear previous text labels

        belief_map = belief_maps(idx, :);
        for col = 1:cols
            for row = 1:rows
                map_idx = (row - 1) * cols + col;
                value = belief_map(map_idx);

                % Assign soft color based on class label
                switch value
                    case 1
                        color = red_soft;
                    case 2
                        color = blue_soft;
                    case 3
                        color = green_soft;
                    otherwise
                        color = [1 1 1];
                end
                set(cell_handles(row, col), 'FaceColor', color);

                % Optional: show cell index (for debugging)
                cx = (col - 0.5) * cell_width;
                cy = (row - 0.5) * cell_height;
                text(cx, cy, num2str(map_idx), 'FontSize', 6, ...
                     'HorizontalAlignment', 'center');
            end
        end
    end

    % === Display frame ===
    title(sprintf('Timestep %d', frame_t));
    drawnow;
    pause(0.00001);
    %frame = getframe(gcf);  % uncomment to save all frames
    %writeVideo(v, frame);

    if all(done)
        break;
    end
end

% === Plot estimated source position (yellow star) ===
expected_path = fullfile(exp_folder, '101', 'expected_a_posteriori.csv');
expected_data = readmatrix(expected_path);
estimated_x = expected_data(end, 2);
estimated_y = expected_data(end, 3);

plot(estimated_x, estimated_y, 'p', 'MarkerSize', 16, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', yellow);

% Save final frame to video
frame = getframe(gcf);
writeVideo(v, frame);
close(v);
disp(['Video saved to: ', output_video_path]);
