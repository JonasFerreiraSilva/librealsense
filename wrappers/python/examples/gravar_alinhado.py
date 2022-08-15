import pyrealsense2 as rs
import numpy as np
import cv2 

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

color_path = 'rgb.avi'
depth_path = 'depth.avi'
colorwriter = cv2.VideoWriter(color_path, cv2.VideoWriter_fourcc(*'XVID'), 30, (640,480), 1)
depthwriter = cv2.VideoWriter(depth_path, cv2.VideoWriter_fourcc(*'XVID'), 30, (640,480), 1)

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

        #confere se a camera ta abrindo so dois canais.
        if not aligned_depth_frame or not color_frame: continue

        #transforma os np arrays em "imagens"
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # adiciona o mapa de cor dentro da camada de profundidade e depois junta a imagem rgb e a do amap de cor em uma unica imagem
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        
        #salva o frame dentro do buffer para gravar o video
        colorwriter.write(color_image)
        depthwriter.write(depth_colormap)
        
       
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
    colorwriter.release()
    depthwriter.release()
    pipeline.stop()