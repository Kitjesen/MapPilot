function test_demo_recovery_escape
%TEST_DEMO_RECOVERY_ESCAPE Verify the public MATLAB visualization interface.

output_dir = fullfile(tempdir, 'lingtu_recovery_escape_test');
if isfolder(output_dir)
    rmdir(output_dir, 's');
end
mkdir(output_dir);
cleanup = onCleanup(@() remove_output(output_dir));

result = demo_recovery_escape(output_dir, false, true);

assert(result.verified, 'The selected recovery path must be verified.');
assert(strcmp(result.action, 'rotate_observe_translate'), ...
    'The blind-exit scene must scan before translating.');
assert(~result.initial_translation_available);
assert(result.rotation_fallback_verified);
assert(result.observation_generation_advanced);
assert(result.path(end, 2) > 0.9, ...
    'The selected escape must reach the robot-left (+Y) outlet.');
assert(abs(result.path(end, 1)) < 0.4, ...
    'The selected escape must remain inside the narrow outlet corridor.');
assert(result.portal_selected, 'The selected path must reach a portal.');
assert(result.path_verified, 'Every footprint along the selected path must be safe.');
assert(result.candidate_count > 0, 'The search must expose evaluated candidates.');
assert(max(abs(result.lidar.origin_body_m - [0.402876074867229, 0, 0.0582019450665819])) < 1e-12);
assert(max(abs(result.lidar.mount_rpy_rad - [-pi, -pi/4, 0])) < 1e-12);
assert(isequal(result.lidar.nominal_vertical_fov_deg, [-7, 52]));
assert(result.lidar.nominal_horizontal_fov_deg == 360);
assert(result.lidar.rear_reference_blind_fraction > 0.99);
assert(result.lidar.rear_reference_blind_fraction <= 1.0);
assert(result.observation.post_scan_observed_free_count > result.observation.initial_observed_free_count);
assert(result.observation.initial_observed_free_count == 0, ...
    'Pre-verified rotation evidence must not be counted as a MID-360 observation.');
assert(result.observation.initial_preverified_safe_count > 0);
assert(result.observation.unknown_count_after_scan > 0);
assert(result.observation.translation_generation == result.observation.post_scan_generation);
assert(strcmp(result.observation.initial_source, 'preverified_rotation_sweep'));
assert(strcmp(result.observation.post_scan_source, 'synthetic_mid360_scan'));
assert(result.observation.rear_post_scan_unknown);
assert(result.observation.side_rear_post_scan_unknown);
assert(result.observation.front_wall_shadow_unknown, ...
    'The synthetic scan must not mark cells behind the front wall free.');
assert(result.observation.right_wall_shadow_unknown, ...
    'The synthetic scan must not mark cells behind the right wall free.');
assert(result.observation.left_exit_observed_free, ...
    'The open left exit must gain synthetic MID-360 observed-free evidence.');
assert(~result.observation.rear_translation_safe);
assert(~result.observation.side_rear_translation_safe);
assert(result.observation.path_avoids_unknown);
assert(result.observation.path_new_area_sensor_observed);
assert(result.observation.non_left_portal_count == 0, ...
    'Walls and unknown space must leave only the left exit portal reachable.');
required_tags = {'recovery-observed-free-layer', 'recovery-unknown-layer', ...
    'recovery-obstacle-layer', 'recovery-preverified-rotation-layer', ...
    'recovery-lidar-origin', ...
    'recovery-lidar-mount-label', 'recovery-rear-blind-sector', ...
    'recovery-observation-generation-label', 'recovery-verified-path'};
assert(isequal(sort(result.graphics.required_tags), sort(required_tags)));
assert(isempty(setdiff(required_tags, result.graphics.present_tags)));
assert(contains(result.lidar.coverage_warning, 'not all-around safe ground coverage'));
assert(isfile(result.overview_file), 'The overview PNG was not generated.');
assert(isfile(result.animation_file), 'The recovery GIF was not generated.');
animation_info = dir(result.animation_file);
assert(animation_info.bytes > 0, 'The recovery GIF must not be empty.');

fprintf('test_demo_recovery_escape: PASS\n');
end

function remove_output(output_dir)
if isfolder(output_dir)
    rmdir(output_dir, 's');
end
end
