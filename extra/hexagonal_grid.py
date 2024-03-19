import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm

def create_hexagon(size):
    # Function to create a single hexagon at the origin
    # size: distance from the center to a vertex

    # Angle between vertices
    theta = np.radians(60)
    # Calculate vertices
    vertices = np.array([
        [np.cos(i * theta), 0, np.sin(i * theta)] for i in range(6)
        ]) * size
    
    # Create lines between adjacent vertices
    lines = [[i, (i + 1) % 6] for i in range(6)]
    
    # Create the hexagon as a line set
    hexagon = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(vertices),
        lines=o3d.utility.Vector2iVector(lines)
        )
    return hexagon

def create_hexagon_grid(rows, cols, hex_size, gap=.01):
    # Create a grid of hexagons
    hexagons = o3d.geometry.LineSet()

    # Distance factors for hexagon grid layout
    dx = 3 / 2 * hex_size
    dz = np.sqrt(3) * hex_size + gap

    for row in range(rows):
        for col in range(cols):
            # Offset for even and odd rows
            offset = 0 if col % 2 == 0 else -hex_size * np.sqrt(3) / 2
            
            # Create a hexagon
            hexagon = create_hexagon(hex_size)
            # Translate hexagon to its position in the grid
            hexagon.translate((col * dx, 0, row * dz + offset))
            
            # Merge hexagons
            hexagons += hexagon

    pts = np.array(hexagons.points)
    sx= np.mean(pts[:,0])
    sz= np.mean(pts[:,2])
    hexagons.translate((-sx,0,-sz))
    
    return hexagons

# Parameters
hex_size = 0.1  # Size of the hexagon
rows = 10  # Number of rows of hexagons
cols = 10  # Number of columns of hexagons

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0]) 

# Create the grid
hex_grid = create_hexagon_grid(rows, cols, hex_size, gap =0.01)
pts = np.array(hex_grid.points)

vis = o3d.visualization.Visualizer()
vis.create_window(width=768, height=768)
vis.add_geometry(hex_grid)
vis.add_geometry(frame)

N = 400
t = 0
dt = 0.025

x = pts[:,0]
y = pts[:,1]
z = pts[:,2]

k = 0.5
f = 0.2
a = 0.5

for i in range(N):

    r = np.sqrt((x)**2+(z)**2)
    y = 0.3 * np.sin(2*np.pi*k*r - 2*np.pi*f*t) * np.exp(-r/a)
    pts[:,1] = y
    hex_grid.points = o3d.utility.Vector3dVector(pts)

    vis.update_geometry(hex_grid)
    vis.poll_events()
    vis.update_renderer()
    t += dt


vis.destroy_window()