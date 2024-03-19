import open3d as o3d
import numpy as np

def create_mesh_grid(grid_size=10, spacing=1.0):
    """
    Creates a mesh grid.

    Args:
    - grid_size: The number of squares along one dimension. The total grid will be grid_size x grid_size.
    - spacing: The size of each square in the grid.
    
    Returns:
    - A TriangleMesh object representing the grid.
    """
    # Initialize lists for vertices and triangles
    vertices = []
    triangles = []

    # Generate vertices
    for i in range(grid_size + 1):
        for j in range(grid_size + 1):
            vertices.append([i * spacing, 0, j * spacing])

    # Generate triangles (two per square)
    for i in range(grid_size):
        for j in range(grid_size):
            # Vertex indices
            v0 = i * (grid_size + 1) + j
            v1 = v0 + 1
            v2 = v0 + (grid_size + 1)
            v3 = v2 + 1

            # First triangle
            triangles.append([v0, v2, v3])
            # Second triangle
            triangles.append([v0, v3, v1])

    # Create the mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.array(vertices))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))

    # Compute normals
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.1, 0.7, 0.1])  # Greenish color

    return mesh

# Create the mesh grid
grid_mesh = create_mesh_grid(grid_size=4, spacing=1.0)

# Visualization
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Mesh Grid Visualization', width=800, height=600)
vis.add_geometry(grid_mesh)

# Adjust view control and camera
view_ctl = vis.get_view_control()
view_ctl.set_front([0, -1, -1])  # Change viewing direction
view_ctl.set_lookat([5, 0, 5])  # Focus on the center of the grid
view_ctl.set_up([0, 1, 0])  # Set the up direction
view_ctl.set_zoom(0.8)  # Adjust zoom

vis.run()
vis.destroy_window()