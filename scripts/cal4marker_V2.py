
# from robotic_drawing.control.robot_arm.yaskawa.mh5l import MH5L
# from robotic_drawing.control.tool.robotiq.gripper_2f85 import Gripper2F85
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
import json
from scipy.spatial.transform import Rotation
from vgn.utils.transform import Transform
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import logging
import cv2
import pyrealsense2 as rs
import os
from typing import Tuple
#os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH") #用來讓pyqt5插件能動,它用於從系統環境變量中移除名為 QT_QPA_PLATFORM_PLUGIN_PATH 的變量
#可以精準擺放
#TODO手臂連線參數
HOST = "192.168.255.10" 
PORT = 11000

SPEED=800
CALIBRATION_PARAMS_SAVE_DIR = Path(__file__).parent / "calibration_params/2024_06_13" 

ArUco_json_filename = "./scripts/quaternion.json"
with open(ArUco_json_filename, 'r') as f:
    loaded_data = json.load(f)
tvec = loaded_data["tvec"]
quaternion = loaded_data["quaternion"]
T_ArUco2Cam_m = Transform(Rotation.from_quat(quaternion), tvec)


#TODO Joint位置 T + 52680 after install bin on flang
INIT_JOINT_CONFIGURE = [
    -126330,
    -18970,
    14748,
    -1553,
    -101441,
    46069 + 52680
]
#1,2,3,4
pos=[[174375,-54,16533,62768,-30326,65378],
      [109160,-6946,10866,-42555,-20866,128429],
      [110836,4460,-46832,-147865,-38785,197236],
      [176587,13056,-41194,129422,-43470,14816]]

#Rz轉90度的旋轉矩陣以及下爪深度
theta = np.radians(90)
T_gripper_Rz_90 = np.array([    
    [np.cos(theta), -np.sin(theta), 0, 0],
    [np.sin(theta), np.cos(theta), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
#能用的
def Img_Place_ArUco_detect(img,intrinsic_matrix: np.array, distortion_coefficients: np.array, Length:int):  #目前比較準的方式
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
                frame = cv2.drawFrameAxes(frame, intrinsic_matrix, distortion_coefficients, rvec, tvec, 30)
                # frame = workspace_AR(frame, tvec[i], rvec[i], intrinsic_matrix, distortion_coefficients, WORKSPACE_SIZE_MM, TSDF_SIZE)
                break
            return tvec[0], rvec[0], frame      

def Estimate_FramePlaceCoord(T_Camera2Gripper: np.array, tcp_pose: list,  T_ArUco2Camera: np.array,index: int,visual=False)-> np.array:  #計算座標
        #tcp_pose = robot_arm.controller.get_robot_pose(tool_num=0) #讀取手臂本身座標
        T_Gripper2Base=TcpPose_TO_Numpy(tcp_pose)
        print(f"TCP POSE mat: {T_Gripper2Base}")

        #T_ArUco2Camera=Transform_TO_Numpy(T_ArUco2Cam_m)  # _m代表Transform 類別的物件
        #T_Grasp2ArUco=self.Transform_TO_Numpy(T_grasp2Aruco_m)  # _m代表Transform 類別的物件
        #print("T_Grasp2ArUco",T_Grasp2ArUco)

        #TODO顯示各個座標系
        if(visual==True):
            tm = TransformManager()
            #tm.add_transform("grasp", "ArUco", T_Grasp2ArUco)
            tm.add_transform("gripper", "robot", T_Gripper2Base)
            tm.add_transform("camera", "gripper", T_Camera2Gripper)
            tm.add_transform("Aruco", "camera", T_ArUco2Camera)
            ax = tm.plot_frames_in("Aruco", s=100)
            ax.set_xlim((-1000, 1000))
            ax.set_ylim((-1000, 1000))
            ax.set_zlim((-1000, 1000))
            plt.show() #顯示座標圖

        #下方是貨架中心的相對偏移位置
        if index == 1:
            T_gripper_Rz_90 = np.array([    
            [1,0, 0, +92.5],
            [0,1, 0, -90],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        elif index == 2:
            T_gripper_Rz_90 = np.array([    
            [1,0, 0, -92.5],
            [0,1, 0, -90],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        elif index == 3:
            T_gripper_Rz_90 = np.array([    
            [1,0, 0, -92.5],
            [0,1, 0, +90],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        elif index == 4:
            T_gripper_Rz_90 = np.array([    
            [1,0, 0, +92.5],
            [0,1, 0, +90],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])

        #TODO base座標計算從右邊往左看,相機座標到夾爪座標再到base座標
        T_Grasp2Base = T_Gripper2Base @ T_Camera2Gripper @ T_ArUco2Camera @ T_gripper_Rz_90
        #T_Grasp2Base = T_Gripper2Base @ T_Camera2Gripper @ T_ArUco2Camera
        print("T_Camera2Gripper", T_Camera2Gripper)
        print("T_Gripper2Base",T_Gripper2Base)
        print("T_ArUco2Camera",T_ArUco2Camera)
        
        return T_Grasp2Base

# def EstimateCoord(T_Camera2Gripper: np.array, robot_arm: MH5L,  T_ArUco2Camera: np.array,index: int,visual=False)-> np.array:  #計算座標
#         tcp_pose = robot_arm.controller.get_robot_pose(tool_num=0) #讀取手臂本身座標
#         print(tcp_pose)
#         T_Gripper2Base=TcpPose_TO_Numpy(tcp_pose)
#         print(f"TCP POSE mat: {T_Gripper2Base}")

#         #T_ArUco2Camera=Transform_TO_Numpy(T_ArUco2Cam_m)  # _m代表Transform 類別的物件
#         #T_Grasp2ArUco=self.Transform_TO_Numpy(T_grasp2Aruco_m)  # _m代表Transform 類別的物件
#         #print("T_Grasp2ArUco",T_Grasp2ArUco)

#         #TODO顯示各個座標系
#         if(visual==True):
#             tm = TransformManager()
#             #tm.add_transform("grasp", "ArUco", T_Grasp2ArUco)
#             tm.add_transform("gripper", "robot", T_Gripper2Base)
#             tm.add_transform("camera", "gripper", T_Camera2Gripper)
#             tm.add_transform("Aruco", "camera", T_ArUco2Camera)
#             ax = tm.plot_frames_in("Aruco", s=100)
#             ax.set_xlim((-1000, 1000))
#             ax.set_ylim((-1000, 1000))
#             ax.set_zlim((-1000, 1000))
#             plt.show() #顯示座標圖

#         if index == 1:
#             T_gripper_Rz_90 = np.array([    
#             [1,0, 0, +92.5],
#             [0,1, 0, -90],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#             ])
#         elif index == 2:
#             T_gripper_Rz_90 = np.array([    
#             [1,0, 0, -92.5],
#             [0,1, 0, -90],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#             ])
#         elif index == 3:
#             T_gripper_Rz_90 = np.array([    
#             [1,0, 0, -92.5],
#             [0,1, 0, +90],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#             ])
#         elif index == 4:
#             T_gripper_Rz_90 = np.array([    
#             [1,0, 0, +92.5],
#             [0,1, 0, +90],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#             ])

#         #TODO base座標計算從右邊往左看,相機座標到夾爪座標再到base座標
#         T_Grasp2Base = T_Gripper2Base @ T_Camera2Gripper @ T_ArUco2Camera @ T_gripper_Rz_90
#         #T_Grasp2Base = T_Gripper2Base @ T_Camera2Gripper @ T_ArUco2Camera
#         print("T_Camera2Gripper", T_Camera2Gripper)
#         print("T_Gripper2Base",T_Gripper2Base)
#         print("T_ArUco2Camera",T_ArUco2Camera)
        
#         return T_Grasp2Base

def Transform_TO_Numpy(T_source2target: Transform) -> np.array:
        Source2Target_r = T_source2target.as_matrix()[:3,:3]
        Source2Target_t = T_source2target.as_matrix()[:3,3]  #公尺
        #T_Source2Target = np.r_[np.c_[Source2Target_r, Source2Target_t], [[0, 0, 0, 1]]]
        T_Source2Target = np.r_[np.c_[Source2Target_r, Source2Target_t*1000], [[0, 0, 0, 1]]] 
        #如使用的是cv2.aruco.estimatePoseSingleMarkers(corners, 0.022, mtx, dist)就要乘以1000
        return T_Source2Target
    
    #TODO Tcp_pose機器人座標轉成4x4numpy
def TcpPose_TO_Numpy(tcp_pose: list) -> np.array:
        tvec = tcp_pose[:3]
        x_angle, y_angle, z_angle = tcp_pose[3:]

        T_Source2Target= np.eye(4)
        T_Source2Target[:3, :3] = Rotation.from_euler(
            "xyz", [x_angle, y_angle, z_angle], degrees=True
        ).as_matrix()
        T_Source2Target[:3, 3] =np.array(tvec).T

        return T_Source2Target


# if __name__ == "__main__":
    # robot_arm = MH5L(host=HOST, port=PORT)

    # with open(str(CALIBRATION_PARAMS_SAVE_DIR / 'calibration_params.json')) as f:
    #     calibration_params = json.load(f)
    #     T_Camera2Gripper = np.array(calibration_params["T_cam2gripper"])

    # pos_list=[]
    # #TODO夾爪控制 
    # #目前註解掉gripper="" 
    # logging.info("Init")
    # gripper = Gripper2F85()
    # logging.info("Connect")
    # success = gripper.connect()
    # logging.info("Reset")
    # success = gripper.reset()
    # gripper.close()
    # position=[1,2,3,4]
    
    # for index in position:
    #     #手臂控制 #目前註解掉
    #     robot_arm = MH5L(host=HOST, port=PORT)
    #     #robot_arm="" 
    #     logging.info("[Robot] Init")
    #     robot_arm.power_on()
    #     logging.info("[Robot] Power on")
    #     robot_arm.move_to_joint_config([144639,-36455,-25636,68992,-10800,59514],SPEED)

    #     robot_arm.move_to_joint_config(pos[index-1], SPEED)
    #     logging.info("[Robot] Move to initial pose by joint configure")
    #     tvec, rvec, aruco_detect_result = aruco_detect()

    #     T_ArUco2Camera= np.eye(4)
    #     T_ArUco2Camera[:3, :3] = Rotation.from_rotvec(rvec).as_matrix()
    #     T_ArUco2Camera[:3, 3] = np.array(tvec[0]).T
    #     T_Grasp2Base = EstimateCoord(T_Camera2Gripper,robot_arm,T_ArUco2Camera,index)
        
    #     pos_list.append(T_Grasp2Base)
        
    #     print("T_Grasp2Base")
    #     print(T_Grasp2Base)
    #     Base_point_t = T_Grasp2Base[:3, 3] #3x1 T
    #     Base_point_r = T_Grasp2Base[:3,:3] #3x3
    #     Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
    #     print("Base_point_t")
    #     print(Base_point_t)
        
    #     robot_arm.move_to_pose(
    #         Tx=Base_point_t[0],
    #         Ty=Base_point_t[1],
    #         Tz=Base_point_t[2],
    #         Rx=89,
    #         Ry=0,
    #         Rz=175,
    #         speed=SPEED)
    #     index+=1
    #     # 指定要保存的文件名

    # ArUco_json_filename = "./scripts/ArUco_Frame/Grid_pos.json"
    # with open(ArUco_json_filename, 'w') as f:
    #     json.dump(np.asarray(pos_list), f)