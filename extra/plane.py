import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm

num=2

xy = np.linspace(start=-1,stop=1,num=num)
X,Y = np.meshgrid(xy,xy)
Z = np.zeros_like(X)

lines = [
        [0, 1], [1, 2], [2, 3], [3, 4],  # Bottom square edges
        [4, 5], [5, 6], [6, 7], [6, 7],
        [0, 4], [1, 5], [2, 6], [3, 7],
    ]

pos =np.array((X.flatten(),Y.flatten(),Z.flatten())).T


membrane = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)

# membrane = o3d.geometry.LineSet(
#         points=o3d.utility.Vector3dVector(pos),
#         lines=o3d.utility.Vector2iVector(lines)
#     )


o3d.visualization.draw([membrane])