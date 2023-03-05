from board import apriltagboard
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
board = apriltagboard.AprilTagBoard("../config/apriltag_real.yml", "../config/tagId.csv")
img = cv2.imread("../real_data/2020_12_12_12_22/0_color.bmp")
board.detectTags(img,verbose=1)