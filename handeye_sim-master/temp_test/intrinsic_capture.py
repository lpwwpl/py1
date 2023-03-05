from camera import kinect
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import numpy as np
import os
from PIL import Image
import time

camera = kinect.Kinect(1026,"../config/intrinsic_real_world.yml")
save_dir = "../real_data"
timestamp = time.time()
timestruct = time.localtime(timestamp)
time_str = time.strftime('%m_%d_%H_%M', timestruct)
save_dir = os.path.join(save_dir,time_str)
os.makedirs(save_dir)
while(True):
    flag, rgb,depth = camera.get_rgb_depth_image()
    if not flag:
        continue
    img_resize = cv2.resize(rgb,(512,512))
    cv2.imshow("rgb",img_resize)
    key = cv2.waitKey(100)
    print(key)
    if key == 115:
        timestamp = time.time()
        timestruct = time.localtime(timestamp)
        time_str = time.strftime('%m_%d_%H_%M_%S', timestruct)
        cv2.imwrite(os.path.join(save_dir,"{}_color.bmp".format(time_str)),rgb)
        depth_image = Image.fromarray(depth)
        depth_image.save(os.path.join(save_dir,"{}_depth.png".format(time_str)))
    if key == 112:
        break
camera.realease()

