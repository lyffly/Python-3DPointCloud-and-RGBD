# coding = utf-8
# coding by liuyunfei
# origin code from librealsense examples(github)

import pyrealsense2 as rs2
import matplotlib.pyplot as plt
import numpy as np
import cv2

try:
    pipe = rs2.pipeline()
    pipe.start()

    while True:
        frames = pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        if depth is None:
            continue
        #print(depth.get_distance(100,200))
        depth_image = np.asarray(depth.get_data())

        color_depth = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.03),cv2.COLORMAP_JET)

        cv2.namedWindow("Demo",cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Demo",color_depth)
        cv2.waitKey(1)
    
    exit(0)
    
except Exception as e:
    print(e)
    pass


