import pyrealsense2 as rs
import numpy as np
import math
import time
import cv2 

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        #Esta função pega um novo conjunto de quadros disponível em um dispositivo
        frames = pipeline.wait_for_frames()
        
        #alinha o quadro de cores ao quadro de profundidade
        aligned_frames =  align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame: continue

        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        point_one = (200,170)
        point_two = (100,400)
        point_three = (440,165)
        point_four = (565,375)

        cv2.circle(color_image, point_one, 4, (0, 0, 255))
        cv2.circle(color_image, point_two, 4, (0, 0, 255))
        cv2.circle(color_image, point_three, 4, (0, 0, 255))
        cv2.circle(color_image, point_four, 4, (0, 0, 255))

        distance_one = depth_image[point_one[1], point_one[0]]
        distance_two = depth_image[point_two[1], point_two[0]]
        distance_three = depth_image[point_three[1], point_three[0]]
        distance_four = depth_image[point_four[1], point_four[0]]
        # print(distance)
        cv2.putText(color_image, "p1 {}mm".format(distance_one), (point_one[0], point_one[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(color_image, "p2 {}mm".format(distance_two), (point_two[0], point_two[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(color_image, "p3 {}mm".format(distance_three), (point_three[0], point_three[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(color_image, "{}mm".format(distance_four), (point_four[0], point_four[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        
        #Usa o valor de pixel da imagem colorida com alinhamento de profundidade para obter eixos 3D
        x, y = 200, 300
        depth = aligned_depth_frame.get_distance(x, y)
        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
        distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
        print("Distance from camera to pixel:", distance)
        print("Z-depth from camera surface to pixel surface:", depth)
        
       
        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

except Exception as e:
    print(e)
    pass

finally:
    pipeline.stop()