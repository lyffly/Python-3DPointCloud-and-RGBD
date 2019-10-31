# coding = utf-8
# coding by liuyunfei
# origin code from librealsense examples(github)
# 需要D435i相机  not done yet

import pyrealsense2 as rs2
import numpy as np
import time
import math


pipe = rs2.pipeline()
config = rs2.config()
#config.enable_stream(rs2.stream.color,1280,720,rs2.format.rgb8,30)
#config.enable_stream(rs2.stream.depth,1280,720,rs2.format.z16,30)
#config.enable_stream(rs2.stream.pose)
config.enable_stream(rs2.stream.accel)
config.enable_stream(rs2.stream.gyro)

profile = pipe.start(config)

try:    
    for i in range(5000):
        t1 = time.time()
        frames = pipe.wait_for_frames()
        #frames.first_or_default()
        pose = frames.get_motion_frame()

        
        if motion is not None:
            data = pose.get_motion_frame()

            print("Frame :",pose.frame_number)            
            print("rotation x:",data.rotation.x)
            print("rotation y:",data.rotation.y)
            print("rotation z:",data.rotation.z)
             
                

        dt = time.time() - t1
        print("[INFO] FPS: ",str(int(1.0/dt)))



except Exception as e:
    print(e)
    pass

finally:
    pipe.stop()
    



