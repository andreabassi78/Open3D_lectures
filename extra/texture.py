import open3d as o3d
from PIL import Image
import numpy as np

body = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
folder = "C:\\Users\\andre\\OneDrive - Politecnico di Milano\\Documenti\\PythonProjects\\Open3D_lectures\\extra\\"

img = np.asarray(Image.open(folder+'/earth.jpg'))
body.textures = [o3d.geometry.Image(img)]

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(body)

N = 200

for i in range(N):

    body.translate(0.01)
    vis.update_geometry(body)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()