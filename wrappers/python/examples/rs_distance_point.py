import cv2
import pyrealsense2 as rs
from realsense_depth import *

dc = DepthCamera()

while True:
    ret, depth_frame, color_frame = dc.get_frame()

    point = (400,300)
    cv2.circle(color_frame, point, 4, (0, 0, 255))
    distance = depth_frame[point[1], point[0]]
    # print(distance)
    cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)


    cv2.imshow("Color Frame", color_frame)
    cv2.imshow("Depth frame", depth_frame)
    key = cv2.waitKey(1)
    if key == 27:
        break