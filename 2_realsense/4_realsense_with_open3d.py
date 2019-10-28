# coding = utf-8
# coding by liuyunfei
# origin code from librealsense examples(github)

import pyrealsense2 as rs2
import numpy as np
import open3d as op3
import time


vis = op3.visualization.Visualizer()
vis.create_window("point cloud",width=1280,height=720)


pipe = rs2.pipeline()
config = rs2.config()
config.enable_stream(rs2.stream.color,1280,720,rs2.format.rgb8,30)
config.enable_stream(rs2.stream.depth,1280,720,rs2.format.z16,30)

profile = pipe.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("深度相机的缩放比： ",depth_scale)

align_to = rs2.stream.color
align = rs2.align(align_to)
add_once = False

pointcloud = op3.geometry.PointCloud()

i = 0

try:    
    while True:
        t1 = time.time()
        frames = pipe.wait_for_frames()
        aligned_frames = align.process(frames)
        profile = frames.get_profile()

        depth = aligned_frames.get_depth_frame()
        color = aligned_frames.get_color_frame()
        if depth is None or color is None:
            continue
        #print(depth.get_distance(100,200))
        depth_image = np.asarray(depth.get_data())
        color_image = np.asarray(color.get_data())

        img_depth = op3.geometry.Image(depth_image)
        img_color = op3.geometry.Image(color_image)
        #depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.03),cv2.COLORMAP_JET)
        
        rgbd_image = op3.geometry.RGBDImage.create_from_color_and_depth(img_color,img_depth,convert_rgb_to_intensity=False)
        
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = op3.camera.PinholeCameraIntrinsic(
                intrinsics.width,intrinsics.height,
                intrinsics.fx,intrinsics.fy,
                intrinsics.ppx,intrinsics.ppy
        )
        pcd = op3.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image,
                pinhole_camera_intrinsic
        )
        pointcloud.points = pcd.points
        pointcloud.colors = pcd.colors
                
        #print(np.asarray(pointcloud.points).shape)
        #print(np.asarray(pointcloud.colors).shape)
        
        if add_once == False:
            vis.add_geometry(pointcloud)
            add_once = True
            
        if i == 60:
            op3.io.write_point_cloud("demo.pcd",pointcloud)
            print("[-------important------] write done")

        i += 1

        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()

        dt = time.time()-t1
        print("[INFO] FPS: ",str(int(1.0/dt)))

   
except Exception as e:
    print(e)
    pass

finally:
    pipe.stop()
    



