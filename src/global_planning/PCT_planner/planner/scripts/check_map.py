import pickle
import numpy as np
import sys
import os

# Adjust path to find pickle
pickle_path = '/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/tomogram/spiral0.3_2.pickle'

if not os.path.exists(pickle_path):
    print(f"Error: {pickle_path} not found")
    sys.exit(1)

with open(pickle_path, 'rb') as f:
    data = pickle.load(f)

resolution = float(data['resolution'])
center = np.asarray(data['center'], dtype=np.double)
tomogram = np.asarray(data['data'], dtype=np.float32)
# shape is usually (layers, properties, height, width) -> (5, 5, 413, 209)

print(f"Resolution: {resolution}")
print(f"Center: {center}")
print(f"Shape: {tomogram.shape}")

# map_dim from planner_wrapper line 47: [tomogram.shape[2], tomogram.shape[3]] -> [413, 209]
map_h, map_w = tomogram.shape[2], tomogram.shape[3]
offset = np.array([map_h / 2, map_w / 2])

# tomogram[0] is Traversability. 
# Let's find some valid points in the middle slice (slice index 2)
slice_idx = 2
trav_map = tomogram[slice_idx, 0, :, :]

# Find indices where traversability is "good" (e.g., > 0.5 or just not 0)
# Assuming typical occupancy grid logic where free space has value
valid_indices = np.argwhere(trav_map > 0.5)

if len(valid_indices) == 0:
    print("No valid traversable points found > 0.5. Checking for any non-zero...")
    valid_indices = np.argwhere(trav_map != 0)

if len(valid_indices) > 0:

    print(f"Found {len(valid_indices)} valid points.")
    
    # Let's find a connected cluster using BFS/DFS to ensure connectivity
    # Simple approach: Pick a seed, find neighbors, verify they are in valid_indices
    
    # 1. Pick a start point (randomly or centrally)
    start_idx_flat = len(valid_indices) // 2
    start_yx = valid_indices[start_idx_flat]
    
    # 2. Find a point that is close but not too close (e.g., 5-10 meters away)
    # distance in pixels: 5m / 0.2res = 25 pixels
    target_dist_px = 25
    
    best_end_yx = None
    min_dist_err = 99999
    
    start_y, start_x = start_yx
    
    for yx in valid_indices:
        dy = yx[0] - start_y
        dx = yx[1] - start_x
        dist = np.sqrt(dy*dy + dx*dx)
        
        err = abs(dist - target_dist_px)
        if err < min_dist_err:
            min_dist_err = err
            best_end_yx = yx
            
    # Calculate World Coordinates
    # pos_y = (idx_y - offset[0]) * resolution + center[1]
    # pos_x = (idx_x - offset[1]) * resolution + center[0]
    
    def get_world_pos(yx):
        y, x = yx
        wy = (y - offset[0]) * resolution + center[1]
        wx = (x - offset[1]) * resolution + center[0]
        return wx, wy

    sx, sy = get_world_pos(start_yx)
    ex, ey = get_world_pos(best_end_yx)
    
    print("-" * 30)
    print(f"Recommended Start (TF): x={sx:.2f}, y={sy:.2f}, z=0.0")
    print(f"Recommended Goal:       x={ex:.2f}, y={ey:.2f}, z=0.0")
    print("-" * 30)
    print("Run these commands:")
    print(f"1. ros2 run tf2_ros static_transform_publisher {sx:.2f} {sy:.2f} 0 0 0 0 map body")
    print(f"2. ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {ex:.2f}, y: {ey:.2f}, z: 0.0}}}}}}\"")

else:
    print("No valid points found in slice 2.")

