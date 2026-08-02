function result = demo_recovery_escape(output_dir, show_window, make_animation)
%DEMO_RECOVERY_ESCAPE Visualize LingTu's autonomous local recovery strategy.
%   demo_recovery_escape() creates a static overview and an animated GIF.
%   The deterministic scene first rejects translation through unknown space,
%   verifies a rotation sweep, projects a newer synthetic MID-360 observation,
%   and only then replans translation through observed-free cells. Native
%   odometry, motor execution, retry limits, and final safety remain contract
%   callouts; this geometry demo does not claim real sensor or motion evidence.
%   This offline explanation is not a replacement for the production planner.

if nargin < 1 || isempty(output_dir)
    script_dir = fileparts(mfilename('fullpath'));
    repo_root = fileparts(fileparts(fileparts(script_dir)));
    output_dir = fullfile(repo_root, 'docs', 'media', 'figures');
end
if nargin < 2 || isempty(show_window), show_window = true; end
if nargin < 3 || isempty(make_animation), make_animation = true; end
if ~isfolder(output_dir), mkdir(output_dir); end

base_scene = unique_exit_scene();
sensor = mid360_mount();
scan_yaw = pi / 2;
rotation_samples = linspace(0, scan_yaw, 7);

% Nominal FOV is not free-space evidence. The initial generation admits only
% the footprint that was already proven safe for the illustrated rotation.
swept_footprint = rotation_swept_mask(base_scene, 0, scan_yaw);
initial_sensor_observed = false(size(swept_footprint));
initial_scene = with_observation(base_scene, initial_sensor_observed, ...
    swept_footprint);
initial_plan = search_translation(initial_scene);
rotation_verified = verify_rotation_sweep(initial_scene, 0, scan_yaw);

post_scan_observed = observation_sweep(base_scene, sensor, rotation_samples);
post_scan_scene = with_observation(base_scene, post_scan_observed, ...
    swept_footprint);
plan = search_translation(post_scan_scene);
assert(~initial_plan.verified, 'LingTu:RecoveryDemo:UnexpectedInitialTranslation');
assert(rotation_verified, 'LingTu:RecoveryDemo:UnsafeRotationFallback');
assert(plan.verified, 'LingTu:RecoveryDemo:NoVerifiedPathAfterObservation');
sensor.rear_reference_blind_fraction = rear_reference_blind_fraction(sensor);
[path_avoids_unknown, path_new_area_sensor_observed] = ...
    path_evidence_checks(post_scan_scene, plan.path);
visibility = 'off';
if show_window, visibility = 'on'; end

overview_file = fullfile(output_dir, 'autonomous_recovery_escape_overview.png');
required_tags = {'recovery-observed-free-layer', 'recovery-unknown-layer', ...
    'recovery-obstacle-layer', 'recovery-preverified-rotation-layer', ...
    'recovery-lidar-origin', ...
    'recovery-lidar-mount-label', 'recovery-rear-blind-sector', ...
    'recovery-observation-generation-label', 'recovery-verified-path'};
[figure_handle, present_tags] = render_overview(base_scene, initial_scene, ...
    post_scan_scene, initial_plan, plan, sensor, scan_yaw, visibility);
if ~show_window
    overview_cleanup = onCleanup(@() close_figure(figure_handle));
end
exportgraphics(figure_handle, overview_file, 'Resolution', 180);
if ~show_window
    close_figure(figure_handle);
    clear overview_cleanup;
end

animation_file = '';
if make_animation
    animation_file = fullfile(output_dir, 'autonomous_recovery_escape.gif');
    render_animation(base_scene, initial_scene, post_scan_scene, plan, ...
        sensor, scan_yaw, animation_file, visibility);
end

observation = struct( ...
    'initial_generation', 1, 'post_scan_generation', 2, ...
    'translation_generation', 2, ...
    'initial_source', 'preverified_rotation_sweep', ...
    'post_scan_source', 'synthetic_mid360_scan', ...
    'initial_observed_free_count', nnz(initial_scene.grid.observed_free), ...
    'initial_preverified_safe_count', nnz(initial_scene.grid.admitted_safe), ...
    'post_scan_observed_free_count', nnz(post_scan_scene.grid.observed_free), ...
    'unknown_count_after_scan', nnz(post_scan_scene.grid.unknown));
observation.rear_post_scan_unknown = ...
    grid_mask_at(post_scan_scene, 'unknown', [-1.0, 0.0]);
observation.side_rear_post_scan_unknown = ...
    grid_mask_at(post_scan_scene, 'unknown', [-0.8, -0.6]);
observation.front_wall_shadow_unknown = ...
    grid_mask_at(post_scan_scene, 'unknown', [1.1, 0.0]);
observation.right_wall_shadow_unknown = ...
    grid_mask_at(post_scan_scene, 'unknown', [0.0, -1.1]);
observation.left_exit_observed_free = ...
    grid_mask_at(post_scan_scene, 'observed_free', [0.0, 1.0]);
observation.rear_translation_safe = ...
    safe_edge(post_scan_scene, [0, 0], [-0.50, 0.0]);
observation.side_rear_translation_safe = ...
    safe_edge(post_scan_scene, [0, 0], [-0.40, -0.40]);
observation.path_avoids_unknown = path_avoids_unknown;
observation.path_new_area_sensor_observed = path_new_area_sensor_observed;
observation.non_left_portal_count = ...
    nnz(plan.portal_points(:, 2) <= 0.8);
graphics = struct('required_tags', {required_tags}, ...
    'present_tags', {present_tags});
result = struct( ...
    'verified', plan.verified, ...
    'action', 'rotate_observe_translate', ...
    'initial_translation_available', initial_plan.verified, ...
    'rotation_fallback_verified', rotation_verified, ...
    'observation_generation_advanced', ...
        observation.post_scan_generation > observation.initial_generation, ...
    'selected_direction_bin', plan.selected_bin, ...
    'portal_selected', plan.portal_selected, ...
    'path_verified', verify_path(post_scan_scene, plan.path), ...
    'candidate_count', plan.candidate_count, ...
    'expanded_node_count', plan.expanded_count, ...
    'path', plan.path, ...
    'lidar', sensor, 'observation', observation, 'graphics', graphics, ...
    'scan_yaw_rad', scan_yaw, ...
    'simulated_scope', ['synthetic LiDAR observation, rotation footprint ' ...
        'verification, generation gate, translation search'], ...
    'contract_callouts', ['native odometry, incremental rotation execution, ' ...
        'retry limits, final motor safety'], ...
    'overview_file', overview_file, ...
    'animation_file', animation_file);
fprintf('Recovery: %s, bin=%d, candidates=%d, expanded=%d\n', ...
    result.action, result.selected_direction_bin, result.candidate_count, ...
    result.expanded_node_count);
fprintf('Overview: %s\n', overview_file);
if make_animation, fprintf('Animation: %s\n', animation_file); end
end

function scene = unique_exit_scene()
p = struct( ...
    'vehicle_length', 0.60, 'vehicle_width', 0.60, ...
    'padding', 0.10, 'height_threshold', 0.20, ...
    'hard_cost', 90.0, 'search_radius', 1.20, ...
    'lattice_resolution', 0.10, 'edge_resolution', 0.05, ...
    'min_translation', 0.35, 'portal_margin', 0.15, ...
    'rotation_step', 0.35, 'clearance_weight', 0.10, ...
    'goal_weight', 0.03);
wall_distance = 0.72;
wall_y = (-wall_distance:0.04:0.84)';
wall_x = (-wall_distance:0.04:wall_distance)';
front = [wall_distance * ones(size(wall_y)), wall_y, 0.30 * ones(size(wall_y))];
rear = [-wall_distance * ones(size(wall_y)), wall_y, 0.30 * ones(size(wall_y))];
right = [wall_x, -wall_distance * ones(size(wall_x)), 0.30 * ones(size(wall_x))];
obstacles = unique([front; rear; right], 'rows');

origin = -1.70;
resolution = 0.10;
grid_size = 34;
centers = origin + ((0:grid_size - 1) + 0.5) * resolution;
risk = 100.0 * ones(grid_size, grid_size);

scene = struct('params', p, 'obstacles', obstacles, 'goal_direction', 0.0, ...
    'grid', struct('risk', risk, 'origin', origin, ...
    'resolution', resolution, 'centers', centers));
end

function sensor = mid360_mount()
sensor = struct( ...
    'origin_body_m', [0.402876074867229, 0.0, 0.0582019450665819], ...
    'mount_rpy_rad', [-pi, -pi / 4, 0.0], ...
    'nominal_horizontal_fov_deg', 360, ...
    'nominal_vertical_fov_deg', [-7, 52], ...
    'reference_plane_z_body_m', 0.0, ...
    'reference_radius_m', 1.0, ...
    'maximum_display_range_m', 1.55, ...
    'model_scope', ...
        'Nominal mount/FOV geometry only; excludes robot self-occlusion and return quality.', ...
    'coverage_warning', ...
        'Nominal 360 deg horizontal FOV is not all-around safe ground coverage.');
end

function rotation = rpy_rotation(rpy)
roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
rotation_x = [1, 0, 0; 0, cos(roll), -sin(roll); ...
    0, sin(roll), cos(roll)];
rotation_y = [cos(pitch), 0, sin(pitch); 0, 1, 0; ...
    -sin(pitch), 0, cos(pitch)];
rotation_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
rotation = rotation_z * rotation_y * rotation_x;
end

function rotation = yaw_rotation(yaw)
rotation = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
end

function observed = observation_sweep(scene, sensor, robot_yaws)
[x_grid, y_grid] = meshgrid(scene.grid.centers, scene.grid.centers);
points = [x_grid(:), y_grid(:), ...
    sensor.reference_plane_z_body_m * ones(numel(x_grid), 1)];
observed = false(size(x_grid));
for yaw = robot_yaws
    visible = points_visible_from_mount(points, sensor, yaw);
    sensor_origin = (yaw_rotation(yaw) * sensor.origin_body_m')';
    visible = apply_obstacle_line_of_sight(scene, sensor_origin(1:2), ...
        points(:, 1:2), visible);
    observed = observed | reshape(visible, size(x_grid));
end
end

function visible = apply_obstacle_line_of_sight(scene, sensor_origin, ...
        target_points, visible)
obstacle_cells = obstacle_cell_mask(scene);
candidate_indices = find(visible)';
sample_step = scene.grid.resolution / 2;
for index = candidate_indices
    segment = target_points(index, :) - sensor_origin;
    sample_count = max(1, ceil(norm(segment) / sample_step));
    for sample = 1:max(sample_count - 1, 0)
        point = sensor_origin + sample / sample_count * segment;
        column = floor((point(1) - scene.grid.origin) / ...
            scene.grid.resolution) + 1;
        row = floor((point(2) - scene.grid.origin) / ...
            scene.grid.resolution) + 1;
        if row < 1 || row > size(obstacle_cells, 1) || ...
                column < 1 || column > size(obstacle_cells, 2) || ...
                obstacle_cells(row, column)
            visible(index) = false;
            break;
        end
    end
end
end

function visible = points_visible_from_mount(points, sensor, robot_yaw)
body_yaw = yaw_rotation(robot_yaw);
sensor_origin = (body_yaw * sensor.origin_body_m')';
sensor_rotation = body_yaw * rpy_rotation(sensor.mount_rpy_rad);
delta_world = points - sensor_origin;
delta_sensor = delta_world * sensor_rotation;
elevation_deg = atan2d(delta_sensor(:, 3), ...
    hypot(delta_sensor(:, 1), delta_sensor(:, 2)));
horizontal_range = hypot(delta_world(:, 1), delta_world(:, 2));
visible = isfinite(elevation_deg) & ...
    elevation_deg >= sensor.nominal_vertical_fov_deg(1) & ...
    elevation_deg <= sensor.nominal_vertical_fov_deg(2) & ...
    horizontal_range <= sensor.maximum_display_range_m;
end

function swept = rotation_swept_mask(scene, start_yaw, end_yaw)
[x_grid, y_grid] = meshgrid(scene.grid.centers, scene.grid.centers);
p = scene.params;
cell_half = scene.grid.resolution / 2;
half_length = p.vehicle_length / 2 + p.padding + cell_half;
half_width = p.vehicle_width / 2 + p.padding + cell_half;
swept = false(size(x_grid));
for yaw = linspace(start_yaw, end_yaw, 37)
    local_x = cos(yaw) * x_grid + sin(yaw) * y_grid;
    local_y = -sin(yaw) * x_grid + cos(yaw) * y_grid;
    swept = swept | ...
        (abs(local_x) <= half_length & abs(local_y) <= half_width);
end
end

function observed_scene = with_observation(scene, sensor_observed, admitted_safe)
if nargin < 3, admitted_safe = false(size(sensor_observed)); end
obstacle_cells = obstacle_cell_mask(scene);
if ~isequal(size(obstacle_cells), size(sensor_observed))
    error('LingTu:RecoveryDemo:ObservationGridShapeMismatch', ...
        'Observation and grid shapes must match.');
end

observed_free = sensor_observed & ~obstacle_cells;
admitted_safe = admitted_safe & ~obstacle_cells & ~observed_free;
safe_evidence = observed_free | admitted_safe;
risk = 100.0 * ones(size(sensor_observed));
risk(safe_evidence) = 8.0;
risk(obstacle_cells) = 95.0;
observed_scene = scene;
observed_scene.grid.risk = risk;
observed_scene.grid.observed_free = observed_free;
observed_scene.grid.admitted_safe = admitted_safe;
observed_scene.grid.safe_evidence = safe_evidence;
observed_scene.grid.obstacle = obstacle_cells;
observed_scene.grid.unknown = ~safe_evidence & ~obstacle_cells;
end

function obstacle_cells = obstacle_cell_mask(scene)
obstacle_cells = false(size(scene.grid.risk));
for index = 1:size(scene.obstacles, 1)
    column = floor((scene.obstacles(index, 1) - scene.grid.origin) / ...
        scene.grid.resolution) + 1;
    row = floor((scene.obstacles(index, 2) - scene.grid.origin) / ...
        scene.grid.resolution) + 1;
    if row >= 1 && row <= size(obstacle_cells, 1) && ...
            column >= 1 && column <= size(obstacle_cells, 2)
        obstacle_cells(row, column) = true;
    end
end
end

function safe = verify_rotation_sweep(scene, start_yaw, end_yaw)
safe = true;
for yaw = linspace(start_yaw, end_yaw, 37)
    if ~safe_oriented_pose(scene, [0, 0], yaw)
        safe = false;
        return;
    end
end
end

function safe = safe_oriented_pose(scene, center, yaw)
p = scene.params;
half_length = p.vehicle_length / 2 + p.padding;
half_width = p.vehicle_width / 2 + p.padding;
delta = scene.obstacles(:, 1:2) - center;
local_x = cos(yaw) * delta(:, 1) + sin(yaw) * delta(:, 2);
local_y = -sin(yaw) * delta(:, 1) + cos(yaw) * delta(:, 2);
admitted = scene.obstacles(:, 3) > p.height_threshold;
if any(admitted & abs(local_x) <= half_length & abs(local_y) <= half_width)
    safe = false;
    return;
end
corners = [half_length, half_width; -half_length, half_width; ...
    -half_length, -half_width; half_length, -half_width];
rotation_2d = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
world_corners = corners * rotation_2d' + center;
grid_max = scene.grid.origin + size(scene.grid.risk, 1) * scene.grid.resolution;
if any(world_corners(:) < scene.grid.origin) || any(world_corners(:) > grid_max)
    safe = false;
    return;
end
[x_grid, y_grid] = meshgrid(scene.grid.centers, scene.grid.centers);
dx = x_grid - center(1); dy = y_grid - center(2);
cell_half = scene.grid.resolution / 2;
local_grid_x = cos(yaw) * dx + sin(yaw) * dy;
local_grid_y = -sin(yaw) * dx + cos(yaw) * dy;
footprint = abs(local_grid_x) < half_length + cell_half & ...
    abs(local_grid_y) < half_width + cell_half;
risk = scene.grid.risk(footprint);
safe = ~isempty(risk) && all(isfinite(risk), 'all') && ...
    all(risk >= 0 & risk <= 100, 'all') && all(risk < p.hard_cost, 'all');
end

function fraction = rear_reference_blind_fraction(sensor)
angles = linspace(-pi, pi, 721)';
points = [sensor.reference_radius_m * cos(angles), ...
    sensor.reference_radius_m * sin(angles), ...
    sensor.reference_plane_z_body_m * ones(size(angles))];
visible = points_visible_from_mount(points, sensor, 0);
rear = cos(angles) < 0;
fraction = nnz(rear & ~visible) / nnz(rear);
end

function value = grid_mask_at(scene, field_name, point)
column = floor((point(1) - scene.grid.origin) / scene.grid.resolution) + 1;
row = floor((point(2) - scene.grid.origin) / scene.grid.resolution) + 1;
if row < 1 || row > size(scene.grid.risk, 1) || ...
        column < 1 || column > size(scene.grid.risk, 2)
    value = true;
    return;
end
mask = scene.grid.(field_name);
value = mask(row, column);
end

function [avoids_unknown, new_area_sensor_observed] = ...
        path_evidence_checks(scene, path)
avoids_unknown = ~isempty(path);
new_area_sensor_observed = ~isempty(path);
if isempty(path), return; end
samples = densify(path, scene.params.edge_resolution);
[x_grid, y_grid] = meshgrid(scene.grid.centers, scene.grid.centers);
half_length = scene.params.vehicle_length / 2 + scene.params.padding;
half_width = scene.params.vehicle_width / 2 + scene.params.padding;
cell_half = scene.grid.resolution / 2;
for index = 1:size(samples, 1)
    footprint = abs(x_grid - samples(index, 1)) < ...
        half_length + cell_half & abs(y_grid - samples(index, 2)) < ...
        half_width + cell_half;
    if any(scene.grid.unknown(footprint), 'all')
        avoids_unknown = false;
    end
    newly_required = footprint & ~scene.grid.admitted_safe;
    if any(newly_required & ~scene.grid.observed_free, 'all')
        new_area_sensor_observed = false;
    end
end
end

function plan = search_translation(scene)
p = scene.params;
half = floor(p.search_radius / p.lattice_resolution + 1.0e-9);
side = 2 * half + 1;
center = half + 1;
visited = false(side, side);
parent = zeros(side, side);
first_bin = -ones(side, side);
queue = zeros(side * side, 1);
root = sub2ind([side, side], center, center);
queue(1) = root;
head = 1;
tail = 1;
visited(center, center) = true;
neighbors = [1, 0; 1, 1; 0, 1; -1, 1; ...
    -1, 0; -1, -1; 0, -1; 1, -1];
blocked = zeros(0, 2);
portal_points = zeros(0, 2);
best_index = 0;
best_portal = false;
best_score = -inf;
candidate_count = 0;
expanded_count = 0;

while head <= tail
    current_index = queue(head);
    head = head + 1;
    expanded_count = expanded_count + 1;
    [current_row, current_column] = ind2sub([side, side], current_index);
    current_ix = current_column - center;
    current_iy = current_row - center;

    for neighbor_index = 1:8
        next_ix = current_ix + neighbors(neighbor_index, 1);
        next_iy = current_iy + neighbors(neighbor_index, 2);
        if abs(next_ix) > half || abs(next_iy) > half, continue; end
        next_x = next_ix * p.lattice_resolution;
        next_y = next_iy * p.lattice_resolution;
        distance = hypot(next_x, next_y);
        if distance > p.search_radius + 1.0e-9, continue; end
        next_row = next_iy + center;
        next_column = next_ix + center;
        if visited(next_row, next_column), continue; end

        direction = first_bin(current_row, current_column);
        if parent(current_row, current_column) == 0
            direction = direction_bin(neighbors(neighbor_index, :));
        end
        from_x = current_ix * p.lattice_resolution;
        from_y = current_iy * p.lattice_resolution;
        if ~safe_edge(scene, [from_x, from_y], [next_x, next_y])
            blocked(end + 1, :) = [next_x, next_y]; %#ok<AGROW>
            continue;
        end

        next_index = sub2ind([side, side], next_row, next_column);
        visited(next_row, next_column) = true;
        parent(next_row, next_column) = current_index;
        first_bin(next_row, next_column) = direction;
        tail = tail + 1;
        queue(tail) = next_index;
        if distance + 1.0e-9 < p.min_translation, continue; end

        candidate_count = candidate_count + 1;
        portal_distance = max(p.portal_margin, 1.5 * p.lattice_resolution);
        portal = distance + portal_distance >= p.search_radius - 1.0e-9;
        heading = atan2(next_y, next_x);
        alignment = cos(wrap_angle(heading - scene.goal_direction));
        clearance = min(p.search_radius, obstacle_clearance(scene, [next_x, next_y]));
        score = distance + p.clearance_weight * clearance + p.goal_weight * alignment;
        if portal, portal_points(end + 1, :) = [next_x, next_y]; end %#ok<AGROW>

        better_class = portal && ~best_portal;
        same_class = portal == best_portal;
        better_score = score > best_score + 1.0e-12;
        tie_break = abs(score - best_score) <= 1.0e-12 && ...
            (best_index == 0 || direction < first_bin(best_index));
        if best_index == 0 || better_class || (same_class && (better_score || tie_break))
            best_index = next_index;
            best_portal = portal;
            best_score = score;
        end
    end
end

path = reconstruct_path(best_index, parent, side, center, p.lattice_resolution);
[rows, columns] = find(visited);
visited_points = [(columns - center) * p.lattice_resolution, ...
    (rows - center) * p.lattice_resolution];
if ~isempty(blocked)
    blocked = unique(round(blocked / p.lattice_resolution) * p.lattice_resolution, 'rows');
end
if ~isempty(portal_points), portal_points = unique(portal_points, 'rows'); end

bins = zeros(8, 1);
initial_safe = false(8, 1);
max_reach = zeros(8, 1);
for index = 1:8
    bins(index) = direction_bin(neighbors(index, :));
    initial_safe(index) = safe_edge(scene, [0, 0], ...
        neighbors(index, :) * p.lattice_resolution);
    mask = visited & first_bin == bins(index);
    [mask_rows, mask_columns] = find(mask);
    if ~isempty(mask_rows)
        max_reach(index) = max(hypot( ...
            (mask_columns - center) * p.lattice_resolution, ...
            (mask_rows - center) * p.lattice_resolution));
    end
end

selected_bin = -1;
if best_index ~= 0, selected_bin = first_bin(best_index); end
plan = struct( ...
    'verified', best_index ~= 0 && verify_path(scene, path), ...
    'path', path, 'selected_bin', selected_bin, ...
    'portal_selected', best_portal, 'selected_score', best_score, ...
    'candidate_count', candidate_count, 'expanded_count', expanded_count, ...
    'visited_points', visited_points, 'blocked_points', blocked, ...
    'portal_points', portal_points, 'direction_bins', bins, ...
    'initial_safe', initial_safe, 'max_reach', max_reach);
end

function path = reconstruct_path(best_index, parent, side, center, resolution)
path = zeros(0, 2);
cursor = best_index;
while cursor ~= 0
    [row, column] = ind2sub([side, side], cursor);
    path = [[(column - center) * resolution, (row - center) * resolution]; path]; %#ok<AGROW>
    cursor = parent(row, column);
end
end

function safe = safe_edge(scene, from_point, to_point)
length_m = norm(to_point - from_point);
sample_count = max(1, ceil(length_m / scene.params.edge_resolution));
safe = true;
for sample = 1:sample_count
    point = from_point + sample / sample_count * (to_point - from_point);
    if ~safe_pose(scene, point)
        safe = false;
        return;
    end
end
end

function safe = safe_pose(scene, point)
p = scene.params;
half_length = p.vehicle_length / 2 + p.padding;
half_width = p.vehicle_width / 2 + p.padding;
epsilon = 1.0e-9;
admitted = scene.obstacles(:, 3) > p.height_threshold;
delta = scene.obstacles(admitted, 1:2) - point;
if any(abs(delta(:, 1)) <= half_length + epsilon & ...
        abs(delta(:, 2)) <= half_width + epsilon)
    safe = false;
    return;
end

grid_max = scene.grid.origin + size(scene.grid.risk, 1) * scene.grid.resolution;
if point(1) - half_length < scene.grid.origin - epsilon || ...
        point(2) - half_width < scene.grid.origin - epsilon || ...
        point(1) + half_length > grid_max + epsilon || ...
        point(2) + half_width > grid_max + epsilon
    safe = false;
    return;
end
cell_half = scene.grid.resolution / 2;
columns = find(abs(scene.grid.centers - point(1)) < half_length + cell_half - epsilon);
rows = find(abs(scene.grid.centers - point(2)) < half_width + cell_half - epsilon);
risk = scene.grid.risk(rows, columns);
safe = all(isfinite(risk), 'all') && all(risk >= 0 & risk <= 100, 'all') && ...
    all(risk < p.hard_cost, 'all');
end

function verified = verify_path(scene, path)
verified = ~isempty(path) && safe_pose(scene, path(1, :));
for index = 2:size(path, 1)
    verified = verified && safe_edge(scene, path(index - 1, :), path(index, :));
    if ~verified, return; end
end
end

function clearance = obstacle_clearance(scene, point)
p = scene.params;
half_length = p.vehicle_length / 2 + p.padding;
half_width = p.vehicle_width / 2 + p.padding;
delta_x = max(abs(scene.obstacles(:, 1) - point(1)) - half_length, 0.0);
delta_y = max(abs(scene.obstacles(:, 2) - point(2)) - half_width, 0.0);
clearance = min(hypot(delta_x, delta_y));
end

function bin = direction_bin(direction)
width = 2 * pi / 16;
angle = atan2(direction(2), direction(1));
if angle < 0, angle = angle + 2 * pi; end
bin = mod(floor((angle + 0.5 * width) / width), 16);
end

function angle = wrap_angle(angle)
angle = mod(angle + pi, 2 * pi) - pi;
end

function [figure_handle, present_tags] = render_overview(base_scene, ...
        initial_scene, post_scan_scene, initial_plan, plan, sensor, ...
        scan_yaw, visibility)
font_name = portable_font();
figure_handle = figure('Color', 'white', 'Visible', visibility, ...
    'Position', [60, 45, 1540, 920], 'Name', 'LingTu autonomous recovery');
layout = tiledlayout(figure_handle, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

candidate_axis = nexttile(layout, 1);
draw_environment(candidate_axis, initial_scene, true);
draw_blind_sector(candidate_axis, sensor);
draw_robot(candidate_axis, [0, 0], base_scene.params, ...
    [0.95, 0.52, 0.08], 0.30, 0);
draw_lidar_top(candidate_axis, sensor, 0, true);
neighbors = [1, 0; 1, 1; 0, 1; -1, 1; ...
    -1, 0; -1, -1; 0, -1; 1, -1];
for index = 1:8
    unit = neighbors(index, :) / norm(neighbors(index, :));
    color = [0.82, 0.16, 0.18];
    if initial_plan.initial_safe(index), color = [0.16, 0.62, 0.28]; end
    quiver(candidate_axis, 0, 0, 0.34 * unit(1), 0.34 * unit(2), 0, ...
        'Color', color, 'LineWidth', 2.3, 'MaxHeadSize', 0.55);
    text(candidate_axis, 0.62 * unit(1), 0.62 * unit(2), ...
        sprintf('bin %d', initial_plan.direction_bins(index)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, ...
        'FontName', font_name, 'Color', color, 'FontWeight', 'bold');
end
title(candidate_axis, '1 先试安全平移：左侧出口仍是 unknown', ...
    'FontName', font_name, 'FontWeight', 'bold');
text(candidate_axis, -1.34, -1.31, sprintf([ ...
    'translation candidate = %d；unknown fail-closed\n', ...
    '灰色扇区由安装姿态 + vertical FOV 推导，不等同实机完整遮挡模型'], ...
    initial_plan.candidate_count), ...
    'FontName', font_name, 'FontSize', 9, 'BackgroundColor', 'white');
text(candidate_axis, -1.34, 1.31, 'observation generation = 1', ...
    'FontName', font_name, 'FontSize', 9, 'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white', 'Tag', 'recovery-observation-generation-label');
text(candidate_axis, -1.34, 1.13, ...
    'amber = pre-verified rotation footprint; not inferred from nominal FOV', ...
    'FontName', font_name, 'FontSize', 8, 'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white');

search_axis = nexttile(layout, 2);
draw_environment(search_axis, post_scan_scene, true);
scatter(search_axis, plan.visited_points(:, 1), plan.visited_points(:, 2), ...
    13, [0.35, 0.74, 0.88], 'filled', 'MarkerFaceAlpha', 0.32);
if ~isempty(plan.blocked_points)
    scatter(search_axis, plan.blocked_points(:, 1), plan.blocked_points(:, 2), ...
        25, [0.84, 0.16, 0.18], 'x', 'LineWidth', 1.0);
end
if ~isempty(plan.portal_points)
    scatter(search_axis, plan.portal_points(:, 1), plan.portal_points(:, 2), ...
        30, [0.18, 0.66, 0.28], 'o', 'LineWidth', 1.0);
end
plot(search_axis, plan.path(:, 1), plan.path(:, 2), '-', ...
    'Color', [0.02, 0.36, 0.88], 'LineWidth', 3.3, ...
    'Tag', 'recovery-verified-path');
sweep_indices = unique(round(linspace(1, size(plan.path, 1), 4)));
for index = sweep_indices
    draw_robot(search_axis, plan.path(index, :), base_scene.params, ...
        [0.12, 0.64, 0.34], 0.10, scan_yaw);
end
draw_robot(search_axis, plan.path(end, :), base_scene.params, ...
    [0.02, 0.40, 0.86], 0.34, scan_yaw);
draw_lidar_top(search_axis, sensor, scan_yaw, false);
draw_rotation_arc(search_axis, scan_yaw);
title(search_axis, sprintf(['2 无安全平移 -> 已验证旋转 -> 等待 generation 2 ' ...
    '-> 重规划：score = %.3f'], ...
    plan.selected_score), 'FontName', font_name, 'FontWeight', 'bold');
text(search_axis, -1.34, -1.31, ...
    '蓝线只经过新鲜 observed-free；旋转本身也必须通过整车扫掠验证', ...
    'FontName', font_name, 'FontSize', 9, 'BackgroundColor', 'white');
text(search_axis, -1.34, 1.31, 'observation generation = 2 (advanced)', ...
    'FontName', font_name, 'FontSize', 9, 'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white', 'Tag', 'recovery-observation-generation-label');

reach_axis = nexttile(layout, 3);
draw_mount_side_view(reach_axis, sensor, font_name);

flow_axis = nexttile(layout, 4);
draw_flow(flow_axis, font_name);
sgtitle(layout, ['LingTu 自主脱困：先验证平移；无安全候选才进入旋转 fallback；' ...
    '新观测到达后才允许重规划平移'], ...
    'FontName', font_name, 'FontSize', 17, 'FontWeight', 'bold');
objects = findall(figure_handle, '-property', 'Tag');
present_tags = get(objects, 'Tag');
if ischar(present_tags), present_tags = {present_tags}; end
present_tags = unique(present_tags(~cellfun('isempty', present_tags)));
end

function draw_environment(axis_handle, scene, show_legend)
hold(axis_handle, 'on');
grid_size = size(scene.grid.risk);
unknown_rgb = repmat(reshape([0.66, 0.68, 0.72], 1, 1, 3), ...
    grid_size(1), grid_size(2), 1);
unknown_handle = imagesc(axis_handle, scene.grid.centers, ...
    scene.grid.centers, unknown_rgb);
unknown_handle.AlphaData = 0.72 * double(scene.grid.unknown);
unknown_handle.Tag = 'recovery-unknown-layer';
admitted_rgb = repmat(reshape([0.98, 0.84, 0.48], 1, 1, 3), ...
    grid_size(1), grid_size(2), 1);
admitted_handle = imagesc(axis_handle, scene.grid.centers, ...
    scene.grid.centers, admitted_rgb);
admitted_handle.AlphaData = 0.72 * double(scene.grid.admitted_safe);
admitted_handle.Tag = 'recovery-preverified-rotation-layer';
observed_rgb = repmat(reshape([0.70, 0.90, 0.76], 1, 1, 3), ...
    grid_size(1), grid_size(2), 1);
observed_handle = imagesc(axis_handle, scene.grid.centers, ...
    scene.grid.centers, observed_rgb);
observed_handle.AlphaData = 0.82 * double(scene.grid.observed_free);
observed_handle.Tag = 'recovery-observed-free-layer';
set(axis_handle, 'YDir', 'normal');
scatter(axis_handle, scene.obstacles(:, 1), scene.obstacles(:, 2), ...
    20, [0.08, 0.08, 0.10], 's', 'filled', ...
    'Tag', 'recovery-obstacle-layer');
rectangle(axis_handle, 'Position', [-scene.params.search_radius, ...
    -scene.params.search_radius, 2 * scene.params.search_radius, ...
    2 * scene.params.search_radius], 'Curvature', [1, 1], ...
    'EdgeColor', [0.32, 0.32, 0.35], 'LineStyle', '--', 'LineWidth', 1.1);
plot(axis_handle, 0, 0, '+', 'Color', [0.10, 0.10, 0.12], ...
    'MarkerSize', 11, 'LineWidth', 1.5);
if show_legend
    text(axis_handle, -1.36, -1.38, ...
        ['green = synthetic observed free   amber = pre-verified rotation ' ...
        'safety   black = obstacle   gray = unknown'], ...
        'FontSize', 8, 'BackgroundColor', 'white', 'Margin', 3);
end
axis(axis_handle, 'equal');
axis(axis_handle, [-1.42, 1.42, -1.42, 1.42]);
grid(axis_handle, 'on');
xlabel(axis_handle, 'Body X / forward (m)');
ylabel(axis_handle, 'Body Y / left (m)');
set(axis_handle, 'Layer', 'top');
end

function draw_robot(axis_handle, center, params, color, alpha_value, yaw)
if nargin < 6, yaw = 0; end
half_length = params.vehicle_length / 2 + params.padding;
half_width = params.vehicle_width / 2 + params.padding;
rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
corners = [half_length, half_width; -half_length, half_width; ...
    -half_length, -half_width; half_length, -half_width] * rotation' + center;
patch(axis_handle, corners(:, 1), corners(:, 2), color, ...
    'FaceAlpha', alpha_value, 'EdgeColor', color, 'LineWidth', 1.5);
heading = rotation * [half_length * 0.78; 0];
quiver(axis_handle, center(1), center(2), heading(1), heading(2), 0, ...
    'Color', color, 'LineWidth', 1.6, 'MaxHeadSize', 0.55);
end

function draw_lidar_top(axis_handle, sensor, robot_yaw, show_label, robot_center)
if nargin < 5, robot_center = [0, 0]; end
body_yaw = yaw_rotation(robot_yaw);
origin = [robot_center, 0]' + body_yaw * sensor.origin_body_m';
rotation = body_yaw * rpy_rotation(sensor.mount_rpy_rad);
forward = rotation * [1; 0; 0];
scatter(axis_handle, origin(1), origin(2), 68, [0.60, 0.08, 0.72], ...
    'd', 'filled', 'MarkerEdgeColor', 'white', 'LineWidth', 0.8, ...
    'Tag', 'recovery-lidar-origin');
quiver(axis_handle, origin(1), origin(2), 0.30 * forward(1), ...
    0.30 * forward(2), 0, 'Color', [0.60, 0.08, 0.72], ...
    'LineWidth', 2.0, 'MaxHeadSize', 0.55);
if show_label
    text(axis_handle, origin(1) + 0.08, origin(2) + 0.12, sprintf([ ...
        'MID-360 body origin = (%.3f, %.3f, %.3f) m\n' ...
        'mount RPY = (-180, -45, 0) deg'], sensor.origin_body_m), ...
        'FontSize', 8, 'BackgroundColor', 'white', 'Margin', 4, ...
        'Tag', 'recovery-lidar-mount-label');
end
end

function draw_blind_sector(axis_handle, sensor)
angles = linspace(-pi, pi, 721)';
points = [sensor.reference_radius_m * cos(angles), ...
    sensor.reference_radius_m * sin(angles), ...
    sensor.reference_plane_z_body_m * ones(size(angles))];
visible = points_visible_from_mount(points, sensor, 0);
rear_blind = cos(angles) < 0 & ~visible;
for sign_value = [-1, 1]
    mask = rear_blind & sign(angles) == sign_value;
    theta = angles(mask);
    if isempty(theta), continue; end
    patch(axis_handle, [0; sensor.reference_radius_m * cos(theta); 0], ...
        [0; sensor.reference_radius_m * sin(theta); 0], ...
        [0.34, 0.35, 0.39], 'FaceAlpha', 0.14, 'EdgeColor', 'none', ...
        'Tag', 'recovery-rear-blind-sector');
end
text(axis_handle, -1.32, 0.02, 'rear low-plane blind sector', ...
    'Rotation', 90, 'HorizontalAlignment', 'center', 'FontSize', 8, ...
    'Color', [0.28, 0.29, 0.32]);
end

function draw_rotation_arc(axis_handle, scan_yaw)
theta = linspace(0, scan_yaw, 60);
radius = 0.55;
plot(axis_handle, radius * cos(theta), radius * sin(theta), '--', ...
    'Color', [0.72, 0.40, 0.06], 'LineWidth', 2.0);
quiver(axis_handle, radius * cos(theta(end)), radius * sin(theta(end)), ...
    -0.15 * sin(theta(end)), 0.15 * cos(theta(end)), 0, ...
    'Color', [0.72, 0.40, 0.06], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
end

function draw_mount_side_view(axis_handle, sensor, font_name)
hold(axis_handle, 'on');
origin = sensor.origin_body_m;
rotation = rpy_rotation(sensor.mount_rpy_rad);
fov = sensor.nominal_vertical_fov_deg;
directions = zeros(3, 2);
for index = 1:2
    beta = deg2rad(fov(index));
    directions(:, index) = rotation * [cos(beta); 0; sin(beta)];
end
ray_length = 0.95;
endpoints = origin' + ray_length * directions;
patch(axis_handle, [origin(1), endpoints(1, 1), endpoints(1, 2)], ...
    [origin(3), endpoints(3, 1), endpoints(3, 2)], ...
    [0.68, 0.82, 1.00], 'FaceAlpha', 0.24, ...
    'EdgeColor', [0.12, 0.42, 0.78], 'LineWidth', 1.5);
scatter(axis_handle, origin(1), origin(3), 72, [0.60, 0.08, 0.72], ...
    'd', 'filled', 'Tag', 'recovery-lidar-origin');
plot(axis_handle, [-1.35, 1.35], ...
    sensor.reference_plane_z_body_m * [1, 1], '-', ...
    'Color', [0.35, 0.36, 0.39], 'LineWidth', 1.1);
rear_point = [-sensor.reference_radius_m, 0, ...
    sensor.reference_plane_z_body_m];
plot(axis_handle, [origin(1), rear_point(1)], ...
    [origin(3), rear_point(3)], '--', 'Color', [0.82, 0.16, 0.18], ...
    'LineWidth', 2.0);
rear_sensor = (rear_point - origin) * rotation;
required_elevation = atan2d(rear_sensor(3), hypot(rear_sensor(1), rear_sensor(2)));
text(axis_handle, -1.28, 0.14, sprintf([ ...
    'rear reference ray requires %.1f deg\n' ...
    '< nominal minimum -7 deg => not observed'], required_elevation), ...
    'FontName', font_name, 'FontSize', 9, 'Color', [0.72, 0.10, 0.12], ...
    'BackgroundColor', 'white');
text(axis_handle, 0.46, 0.70, sprintf([ ...
    'origin = [%.3f, %.3f, %.3f] m\nRPY = [-180, -45, 0] deg\n' ...
    'nominal H-FOV 360 deg != all-around safety'], sensor.origin_body_m), ...
    'FontName', font_name, 'FontSize', 9, 'BackgroundColor', 'white', ...
    'Tag', 'recovery-lidar-mount-label');
xlabel(axis_handle, 'Body X / forward (m)');
ylabel(axis_handle, 'Body Z (m)');
title(axis_handle, '3 安装姿态 + V-FOV [-7, +52] deg 的侧视几何', ...
    'FontName', font_name, 'FontWeight', 'bold');
axis(axis_handle, 'equal');
axis(axis_handle, [-1.35, 1.35, -0.18, 0.88]);
grid(axis_handle, 'on');
end

function draw_flow(axis_handle, font_name)
axis(axis_handle, [0, 10, 0, 10]);
axis(axis_handle, 'off');
hold(axis_handle, 'on');
flow_box(axis_handle, [0.2, 7.8, 2.2, 1.2], {'Normal path', 'unavailable'}, ...
    [0.92, 0.92, 0.94], font_name);
flow_box(axis_handle, [3.0, 7.8, 2.4, 1.2], {'Translation first', 'BFS + footprint'}, ...
    [0.82, 0.91, 1.00], font_name);
flow_box(axis_handle, [6.0, 7.8, 3.3, 1.2], {'Fresh observed-free?', 'unknown = reject'}, ...
    [0.82, 0.91, 1.00], font_name);
flow_box(axis_handle, [6.0, 5.0, 3.3, 1.2], {'Execute translation', 'portal-selected path'}, ...
    [0.78, 0.94, 0.82], font_name);
flow_box(axis_handle, [3.0, 5.0, 2.4, 1.2], {'Verify rotation', 'swept footprint'}, ...
    [1.00, 0.91, 0.68], font_name);
flow_box(axis_handle, [0.2, 5.0, 2.2, 1.2], {'Rotate fallback', 'only after verify'}, ...
    [1.00, 0.91, 0.68], font_name);
flow_box(axis_handle, [0.2, 2.0, 2.2, 1.2], {'Wait for newer', 'observation generation'}, ...
    [1.00, 0.91, 0.68], font_name);
flow_box(axis_handle, [3.0, 2.0, 2.4, 1.2], {'Replan translation', 'new evidence only'}, ...
    [0.82, 0.91, 1.00], font_name);
flow_box(axis_handle, [6.0, 2.0, 3.3, 1.2], {'Final safety / STOP', 'native execution gate'}, ...
    [1.00, 0.80, 0.80], font_name);
flow_arrow(axis_handle, [2.4, 8.4], [3.0, 8.4], [0.25, 0.25, 0.28]);
flow_arrow(axis_handle, [5.4, 8.4], [6.0, 8.4], [0.25, 0.25, 0.28]);
flow_arrow(axis_handle, [7.65, 7.8], [7.65, 6.2], [0.16, 0.55, 0.26]);
flow_arrow(axis_handle, [6.0, 8.0], [5.4, 5.8], [0.72, 0.48, 0.10]);
flow_arrow(axis_handle, [3.0, 5.6], [2.4, 5.6], [0.72, 0.48, 0.10]);
flow_arrow(axis_handle, [1.3, 5.0], [1.3, 3.2], [0.72, 0.48, 0.10]);
flow_arrow(axis_handle, [2.4, 2.6], [3.0, 2.6], [0.25, 0.25, 0.28]);
flow_arrow(axis_handle, [5.4, 2.6], [6.0, 2.6], [0.25, 0.25, 0.28]);
flow_arrow(axis_handle, [7.65, 5.0], [7.65, 3.2], [0.25, 0.25, 0.28]);
text(axis_handle, 6.05, 7.32, 'YES', 'Color', [0.10, 0.50, 0.22], ...
    'FontName', font_name, 'FontWeight', 'bold');
text(axis_handle, 5.35, 6.35, 'NO', 'Color', [0.68, 0.36, 0.06], ...
    'FontName', font_name, 'FontWeight', 'bold');
title(axis_handle, '4 Recovery 状态机：旋转后不能沿用旧观测', ...
    'FontName', font_name, 'FontWeight', 'bold');
end

function flow_box(axis_handle, position, lines, color, font_name)
rectangle(axis_handle, 'Position', position, 'Curvature', 0.08, ...
    'FaceColor', color, 'EdgeColor', color * 0.68, 'LineWidth', 1.4);
text(axis_handle, position(1) + position(3) / 2, ...
    position(2) + position(4) / 2, lines, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontName', font_name, 'FontSize', 9, 'FontWeight', 'bold');
end

function flow_arrow(axis_handle, start_point, end_point, color)
delta = end_point - start_point;
quiver(axis_handle, start_point(1), start_point(2), delta(1), delta(2), 0, ...
    'Color', color, 'LineWidth', 1.8, 'MaxHeadSize', 0.48);
end

function render_animation(base_scene, initial_scene, post_scan_scene, plan, ...
        sensor, scan_yaw, animation_file, visibility)
if isfile(animation_file), delete(animation_file); end
dense_path = densify(plan.path, 0.04);
if size(dense_path, 1) > 28
    dense_path = dense_path(unique(round(linspace(1, size(dense_path, 1), 28))), :);
end
rotation_yaws = linspace(0, scan_yaw, 8);
rotation_frame_count = numel(rotation_yaws);
wait_frame = rotation_frame_count + 1;
total_frames = wait_frame + size(dense_path, 1);
figure_handle = figure('Color', 'white', 'Visible', visibility, ...
    'Position', [150, 70, 900, 820], 'Name', 'LingTu recovery animation');
temporary_frame = fullfile(fileparts(animation_file), 'recovery_frame_tmp.png');
if strcmp(visibility, 'off')
    figure_cleanup = onCleanup(@() close_figure(figure_handle));
end

for frame = 1:total_frames
    temporary_frame_cleanup = onCleanup(@() delete_file_if_present(temporary_frame));
    font_name = portable_font();
    clf(figure_handle);
    axis_handle = axes(figure_handle, 'Position', [0.09, 0.12, 0.84, 0.80]);
    if frame <= rotation_frame_count
        robot_yaw = rotation_yaws(frame);
        frame_scene = initial_scene;
        robot_center = [0, 0];
        generation = 1;
        stage = 'ROTATE_FALLBACK (generation 2 not committed yet)';
        progress = 0;
    elseif frame == wait_frame
        robot_yaw = scan_yaw;
        frame_scene = post_scan_scene;
        robot_center = [0, 0];
        generation = 2;
        stage = 'WAIT_NEW_OBSERVATION -> generation advanced';
        progress = 0;
    else
        path_frame = frame - wait_frame;
        robot_yaw = scan_yaw;
        frame_scene = post_scan_scene;
        robot_center = dense_path(path_frame, :);
        generation = 2;
        stage = 'TRANSLATE (replanned on fresh evidence)';
        progress = (path_frame - 1) / max(size(dense_path, 1) - 1, 1);
    end
    draw_environment(axis_handle, frame_scene, false);
    scatter(axis_handle, plan.visited_points(:, 1), plan.visited_points(:, 2), ...
        9, [0.45, 0.77, 0.88], 'filled', 'MarkerFaceAlpha', 0.18);
    plot(axis_handle, plan.path(:, 1), plan.path(:, 2), '--', ...
        'Color', [0.20, 0.46, 0.82], 'LineWidth', 1.4);
    if frame > wait_frame
        path_frame = frame - wait_frame;
        plot(axis_handle, dense_path(1:path_frame, 1), ...
            dense_path(1:path_frame, 2), '-', ...
            'Color', [0.02, 0.38, 0.88], 'LineWidth', 3.4);
    else
        draw_rotation_arc(axis_handle, robot_yaw);
    end
    draw_robot(axis_handle, robot_center, base_scene.params, ...
        [0.95, 0.48, 0.08], 0.36, robot_yaw);
    draw_lidar_top(axis_handle, sensor, robot_yaw, false, robot_center);
    rectangle(axis_handle, 'Position', [-1.32, -1.34, 2.64, 0.10], ...
        'FaceColor', [0.90, 0.90, 0.92], 'EdgeColor', [0.55, 0.55, 0.58]);
    rectangle(axis_handle, 'Position', [-1.32, -1.34, 2.64 * progress, 0.10], ...
        'FaceColor', [0.10, 0.62, 0.30], 'EdgeColor', 'none');
    text(axis_handle, -1.31, 1.33, sprintf([ ...
        '%s\nobservation_generation = %d\nunknown = fail closed\n' ...
        'demo progress = %3.0f%%'], stage, generation, 100 * progress), ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas', ...
        'FontSize', 10, 'BackgroundColor', 'white', 'Margin', 7, ...
        'Color', [0.08, 0.18, 0.25]);
    title(axis_handle, ['LingTu recovery: translation first; verified rotation ' ...
        'fallback; fresh observation before motion'], ...
        'FontName', font_name, 'FontSize', 15, 'FontWeight', 'bold');
    drawnow;
    exportgraphics(figure_handle, temporary_frame, 'Resolution', 120);
    rgb = imread(temporary_frame);
    [indexed, color_map] = rgb2ind(rgb, 256);
    delay = 0.16;
    if frame == wait_frame, delay = 0.85; end
    if frame == total_frames, delay = 0.90; end
    if frame == 1
        imwrite(indexed, color_map, animation_file, 'gif', ...
            'LoopCount', inf, 'DelayTime', delay);
    else
        imwrite(indexed, color_map, animation_file, 'gif', ...
            'WriteMode', 'append', 'DelayTime', delay);
    end
end
delete_file_if_present(temporary_frame);
clear temporary_frame_cleanup;
if strcmp(visibility, 'off')
    close_figure(figure_handle);
    clear figure_cleanup;
end
end

function dense_path = densify(path, maximum_step)
dense_path = path(1, :);
for index = 2:size(path, 1)
    start_point = path(index - 1, :);
    end_point = path(index, :);
    samples = max(1, ceil(norm(end_point - start_point) / maximum_step));
    for sample = 1:samples
        dense_path(end + 1, :) = start_point + ...
            sample / samples * (end_point - start_point); %#ok<AGROW>
    end
end
end

function font_name = portable_font()
font_name = 'Helvetica';
installed_fonts = listfonts;
candidates = {'Microsoft YaHei', 'Noto Sans CJK SC', 'Noto Sans CJK', ...
    'WenQuanYi Zen Hei', 'Arial Unicode MS', 'DejaVu Sans'};
for index = 1:numel(candidates)
    if any(strcmpi(installed_fonts, candidates{index}))
        font_name = candidates{index};
        return;
    end
end
end

function close_figure(figure_handle)
if isgraphics(figure_handle)
    close(figure_handle);
end
end

function delete_file_if_present(file_path)
if isfile(file_path)
    delete(file_path);
end
end
