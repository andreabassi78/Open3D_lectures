import open3d as o3d
import open3d.core as o3c
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

# Only needed for tutorial, monkey patches visualization
sys.path.append("..")
import open3d_tutorial as o3dtut
# Change to True if you want to interact with the visualization windows
o3dtut.interactive = not "CI" in os.environ