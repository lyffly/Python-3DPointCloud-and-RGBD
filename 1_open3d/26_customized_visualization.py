# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3
import matplotlib.pyplot as plt
import copy
import time
from trajectory_io import *
import os
import sys

def custom_draw_geometry(pcd):
    vis = op3.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

def custom_draw_geometry_load_option(pcd):
    vis = op3.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.get_render_option().load_from_json("demodata/renderoption.json")
    vis.run()
    vis.destroy_window()

def custom_draw_geometry_with_custom_fov(pcd,fov_step):
    vis = op3.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    ctl = vis.get_view_control()
    print(ctl.get_field_of_view())
    ctl.change_field_of_view(step=fov_step)
    print(ctl.get_field_of_view())
    vis.run()
    vis.destroy_window()

def custom_draw_geometry_with_rotation(pcd):
    
    def rotate_view(vis):
        ctr = vis.get_view_control()
        ctr.rotate(10.0,0.0)
        return False
    op3.visualization.draw_geometries_with_animation_callback([pcd],rotate_view)

def custom_draw_geometry_with_key_callback(pcd):

    def change_background_to_black(vis):
        opt = vis.get_render_option()
        opt.background_color= np.asarray([0,0,0])
        return False
    def load_render_option(vis):
        vis.get_render_option().load_from_json("demodata/renderoption.json")
        return False
    def capture_depth(vis):
        depth = vis.capture_depth_float_buffer()
        plt.imshow(np.asarray(depth))
        plt.show()
        return False
    def capture_image(vis):
        image = vis.capture_screen_float_buffer()
        plt.imshow(np.asarray(image))
        plt.show()
        return False
    key_to_callback ={}
    key_to_callback[ord("K")] = change_background_to_black
    key_to_callback[ord("R")] = load_render_option
    key_to_callback[ord(",")] = capture_depth
    key_to_callback[ord(".")] = capture_image
    op3.visualization.draw_geometries_with_key_callbacks([pcd],key_to_callback)
    
def custom_draw_geometry_with_camera_trajectory(pcd):
    custom_draw_geometry_with_camera_trajectory.index = -1
    custom_draw_geometry_with_camera_trajectory.trajectory=\
        op3.io.read_pinhole_camera_trajectory("demodata/camera_trajectory.json")
    custom_draw_geometry_with_camera_trajectory.vis = op3.visualization.Visualizer()

    if not os.path.exists("demodata/image/"):
        os.makedirs("demodata/image")
    if not os.path.exists("demodata/depth/"):
        os.makedirs("demodata/depth")
    
    def move_forward(vis):
        ctr = vis.get_view_control()
        glb = custom_draw_geometry_with_camera_trajectory
        if glb.index >=0:
            print("Capture images {:05d}".format(glb.index))
            depth = vis.capture_depth_float_buffer(False)
            image = vis.capture_screen_float_buffer(False)
            plt.imsave("demodata/depth/{:05d}.png".format(glb.index),np.asarray(depth),dpi=1)
            plt.imsave("demodata/image/{:05d}.png".format(glb.index),np.asarray(image),dpi=1)
        glb.index = glb.index +1
        if glb.index <len(glb.trajectory.parameters):
            ctr.convert_from_pinhole_camera_parameters(
                glb.trajectory.parameters[glb.index]
            )
        else:
            custom_draw_geometry_with_camera_trajectory.vis.register_animation_callback(None)
        return False

    vis = custom_draw_geometry_with_camera_trajectory.vis
    vis.create_window()
    vis.add_geometry(pcd)
    vis.get_render_option().load_from_json("demodata/renderoption.json")
    vis.register_animation_callback(move_forward)
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    pcd = op3.io.read_point_cloud("demodata/fragment.ply")

    print("1. Customized visualization to mimic DrawGeometry")
    #custom_draw_geometry(pcd)

    #custom_draw_geometry_load_option(pcd)

    #custom_draw_geometry_with_custom_fov(pcd,-90.0)

    #custom_draw_geometry_with_rotation(pcd)

    #custom_draw_geometry_with_key_callback(pcd)
    
    custom_draw_geometry_with_camera_trajectory(pcd)
