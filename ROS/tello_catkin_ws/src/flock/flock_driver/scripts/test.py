import numpy as np
import trimesh
from trimesh.viewer.windowed import SceneViewer
import time
import pyglet
import subprocess

#time.sleep(30)
# load a file by name or from a buffer

file = open("pc.off","r")

mesh = trimesh.load_mesh(file, file_type='off')
mesh.show()