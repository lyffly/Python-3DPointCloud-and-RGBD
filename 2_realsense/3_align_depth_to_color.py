# coding = utf-8
# coding by liuyunfei
# origin code from librealsense examples(github)

import pyrealsense2 as rs2
import matplotlib.pyplot as plt
import numpy as np
import cv2

pipe = rs2.pipeline()
config = rs2.config()
config.enable_stream(rs2.stream.color,1280,720,rs2.format.bgr8,30)
config.enable_stream(rs2.stream.depth,1280,720,rs2.format.z16,30)

profile = pipe.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("深度相机的缩放比： ",depth_scale)

align_to = rs2.stream.color
align = rs2.align(align_to)


try:
    
    while True:
        frames = pipe.wait_for_frames()
        aligned_frames = align.process(frames)

        depth = aligned_frames.get_depth_frame()
        color = aligned_frames.get_color_frame()
        if depth is None or color is None:
            continue
        #print(depth.get_distance(100,200))
        depth_image = np.asarray(depth.get_data())
        color_image = np.asarray(color.get_data())

        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.03),cv2.COLORMAP_JET)

        cv2.namedWindow("Demo",cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Demo",depth_color)
        

        cv2.namedWindow("color",cv2.WINDOW_AUTOSIZE)
        cv2.imshow("color",color_image)
        
        key = cv2.waitKey(1)
        if key & 0xFF==ord('q') or key==27 :
            cv2.destroyAllWindows()       

    
except Exception as e:
    print(e)
    pass

finally:
    pipe.stop()
    cv2.destroyAllWindows()



