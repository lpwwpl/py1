import pyrealsense2 as rs
from UtilSet import executable_path
import numpy as np
import cv2
import time
import os
class CamHandler:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pf = self.pipeline.start(self.config)
        self.color_frame = ''
        self.depth_frame = ''
        self.color_img = ''
        self.depth_img = ''
        self.exec_path = executable_path()

        depth_sensor = self.pf.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(self.depth_scale)

    def getColoredImg(self):
        self.color_frame = self.frame.get_color_frame()
        self.color_img = np.asanyarray(self.color_frame.get_data())

    def getDepthImg(self):
        self.depth_frame = self.frame.get_depth_frame()
        self.depth_img = np.asanyarray(self.depth_frame.get_data())
        # print(self.depth_img)

    def getFrame(self):
        self.frame = self.pipeline.wait_for_frames()
        self.getColoredImg()
        self.getDepthImg()

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_img, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_img.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(self.color_img, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((self.color_img, depth_colormap))

        ct = time.time()
        local_time = time.localtime(ct)
        timestamp = time.strftime("%Y%m%d_%H%M%S", local_time)
        colorFileName = os.path.abspath(os.path.join(self.exec_path, "AIQA/picslx/frame-{}.color.png".format(timestamp)))
        depthFileName = os.path.abspath(os.path.join(self.exec_path, "AIQA/picslx/frame-{}.depth.png".format(timestamp)))

    def getIntrinsicArray(self):
        profile = self.pipeline.get_active_profile()
        # print('profile',profile)
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()

        return [color_intrinsics.fx, 0, color_intrinsics.ppx, 0, color_intrinsics.fy, color_intrinsics.ppy, 0, 0, 1]

    def stop_cam(self):
        self.pipeline.stop()