#!/usr/bin/env python

import numpy as np
import trimesh
from trimesh.viewer.windowed import SceneViewer
import time
import pyglet
import subprocess

#time.sleep(30)
# load a file by name or from a buffer

file = open("/home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/scripts/pc.off","r")

mesh = trimesh.load_mesh(file, file_type='off')
scene = trimesh.Scene(geometry=mesh)    

# to keep the raw data intact, disable any automatic processing
#mesh = trimesh.load_mesh('../models/featuretype.STL', process=False)
# is the current mesh watertight?

view = SceneViewer(scene,start_loop=False)
#view.callback = update(view)
view.toggle_culling()
view._gl_enable_lighting = False

def renew_surface():
    subprocess.call("/home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/scripts/powercrust -m 1000000 -i /home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/scripts/pc.pts", shell=True)
    time.sleep(0.01)

def update(dt):
    renew_surface()
    file = open("/home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/scripts/pc.off","r")
    mesh = trimesh.load_mesh(file, file_type='off')
    scene2 = trimesh.Scene(geometry=mesh)
    view.scene = scene2
    view._update_meshes()
    view._update_vertex_list()
    view.cleanup_geometries()
    view.on_draw()
    view.reset_view()
    

pyglet.clock.schedule_interval(update, 5)
pyglet.app.run() 
    
