import cv2
import numpy as np
import time
import pyrealsense2 as rs
import json
from scipy.spatial.transform import Rotation as R
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

#使用一張影像計算四元數
def Img_Pick_ArUco_detect(img,intrinsic_matrix: np.array, distortion_coefficients: np.array, Length:int)-> dict:

#    如果找不打id
    ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    ARUCO_PARAMETERS = cv2.aruco.DetectorParameters()
    #ARUCO_MARK_LENGTH = 22  #mm
    while True:
        # frames = pipeline.wait_for_frames()
        # color_frame = frames.get_color_frame()
        # image = np.asanyarray(color_frame.get_data())
        frame = img.copy()       
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, ARUCO_DICTIONARY, parameters=ARUCO_PARAMETERS)
        rvec, tvec = None, None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                print(f"intrinsic_matrix: {intrinsic_matrix}")
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], Length, intrinsic_matrix, distortion_coefficients)
                frame = cv2.drawFrameAxes(frame, intrinsic_matrix, distortion_coefficients, rvec, tvec, Length)
                # frame = workspace_AR(frame, tvec[i], rvec[i], intrinsic_matrix, distortion_coefficients, WORKSPACE_SIZE_MM, TSDF_SIZE)
                break

            ###### DRAW ID #####
            cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            print(rvec[0])
            rotation_m = R.from_rotvec(rvec[0])  #rotation_m 是transform 類別

            quaternion = rotation_m.as_quat()
            print("四元數")
            print(quaternion)
            print("-----------------")

            data = {
                        "tvec": tvec[0].squeeze().tolist(),
                        "quaternion": quaternion.squeeze().tolist(),
                    }
                    
            # 指定要保存的文件名
            ArUco_json_filename = "./scripts/quaternion.json"

            with open(ArUco_json_filename, 'w') as f:
                json.dump(data, f)

            filename = "./scripts/Saved_IMG/PickArUco.jpg"
            cv2.imwrite(filename, frame)
            # cv2.imshow("Pickplane",frame)
            # cv2.waitKey(0)
            return data


#不透過ros使用realsense取像
def Camstream_ArUco_detect(): 
    distortion=np.array([ 4.90913179e-02 , 5.22699002e-01, -2.65209452e-03  ,1.13033224e-03,-2.17133474e+00])

    Intrinsic=np.array([[625.002 ,  0.      ,   320.751],[  0.     ,    625.002 ,233.084],[  0.    ,       0.    ,       1.        ]])


    # 初始化RealSense相机
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 开始采集
    pipeline.start(config)

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # 将帧数据转换为OpenCV格式
        frame = np.asanyarray(color_frame.get_data())
        cv2.imshow('RealSense Color Image', frame)

        # 检测按键，如果按下q键则退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters =  cv2.aruco.DetectorParameters()
        '''
        detectMarkers(...)
            detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
            mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''

        detector = cv2.aruco.ArucoDetector(cv2.aruco_dict , parameters)
        corners, ids, rejected_img_points = detector.detectMarkers(frame)

        if ids is not None:  
            #TODO 擺放的maker長度是2.2cm使用22mm
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.035, Intrinsic, distortion)
            (rvec-tvec).any() # get rid of that nasty numpy value array error

            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(frame, Intrinsic, distortion, rvec, tvec, 0.05) 
                cv2.aruco.drawDetectedMarkers(frame, corners)
            ###### DRAW ID #####
            cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            print(rvec.squeeze())
            rotation_m = R.from_rotvec(rvec.squeeze())  #rotation_m 是transform 類別

            quaternion = rotation_m.as_quat()
            print("四元數")
            print(quaternion)
            print("-----------------")
        else:
            ##### DRAW "NO IDS" #####
            cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        data = {
                "tvec": tvec.squeeze().tolist(),
                "quaternion": quaternion.tolist(),
            }
            
        # 指定要保存的文件名
        ArUco_json_filename = "./scripts/quaternion.json"

        with open(ArUco_json_filename, 'w') as f:
            json.dump(data, f)

        # 显示结果框架
        cv2.imshow("frame",frame)

        key = cv2.waitKey(1)

        if key == 27:         # 按esc键退出
            print('esc break...')
            cv2.destroyAllWindows()
            pipeline.stop()
            break

        if key == ord(' '):   # 按空格键保存
            filename = str(time.time())[:10] + ".jpg"
            cv2.imwrite(filename, frame)
    
        cv2.waitKey(0)

if __name__ == "__main__":
    Camstream_ArUco_detect()
    