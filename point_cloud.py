import open3d as o3d
import numpy as np

cloud = np.random.uniform(low=-1.0, high=1.0, size=[100,3])
pcd = o3d.t.geometry.PointCloud(cloud)
o3d.visualization.draw_geometries([pcd])