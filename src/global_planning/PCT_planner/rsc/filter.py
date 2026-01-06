import open3d as o3d
 
# 加载点云数据
pcd = o3d.io.read_point_cloud("pcd/scans.pcd")
 
# 计算法线（区域生长需要法线或颜色）
pcd.estimate_normals()

# 执行区域生长分割
# 参数说明：
# - nb_neighbors: 邻域点数
# - nb_points: 最小区域点数
# - smoothness_threshold_deg: 法线角度阈值（度）
labels = pcd.cluster_dbscan(eps=0.02, min_points=10)

# 可视化分割结果
max_label = max(labels) + 1
colors = [[0.5, 0.5, 0.5] for _ in range(len(pcd.points))]
for i in range(len(pcd.points)):
    if labels[i] >= 0:
        colors[i] = [0, 0, labels[i]/max_label]
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])
