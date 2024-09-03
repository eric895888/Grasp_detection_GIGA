#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import json
import logging
from pathlib import Path
from robot.controller import Controller
from robot.gripper import Gripper
import numpy as np
import argparse
from pathlib import Path
import sensor_msgs.msg
from ui.UI import Ui_MainWindow
from vgn import vis
from vgn.experiments.clutter_removal import State
from vgn.detection import VGN
from vgn.perception import *
from vgn.utils import ros_utils                                                                                                     
from vgn.utils.transform import Rotation, Transform
import os
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr                                                                                                                               
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
from vgn.detection_implicit import VGNImplicit
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from cal4marker_V2 import *
from ArUco_detection import *
from typing import Tuple

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH") #出現xcb錯誤時就可以把註解去掉
sys.path.append(os.path.dirname(__file__) + os.sep)  #os.sep 表示当前操作系统的路径分隔符，如在 Windows 上是 \，在 Unix 上是 /。
#Aruco擺放位置
#position=[125.605,-331.208,17.958,178.6843,-0.6676,-0.1229]
#joint=[-104484,-31812,-49428,-832,-62664,90985]
#使用的物件名稱

######################
# object_name="Block" # ["Block", "TT_cube", "Bolt", "Sundries"]
#object_name="TT_cube" # ["Block", "TT_cube", "Bolt", "Sundries"]
#object_name="Bolt" # ["Block", "TT_cube", "Bolt", "Sundries"]
object_name="Block" # ["Block", "TT_cube", "Bolt", "Sundries"]
######################

PLACE_JOINTS={
    0:[-15662,16560,15707,1585,-81132,58007],
    1:[11886,18580,17724,2187,-81985,47525]
}
PLACE_NUMBER=2
#TODO   表示z軸向下爪幾公分單位是mm
OFFSET = {   
        "Block": 70,
        "TT_cube": 90,  #太重最好抓深一點
        "Bolt": 70,
        "Sundries":80
    }
MODEL_PATH = {
        "Block": "./scripts/data/models/Block_giga.pt",
        "TT_cube": "./scripts/data/models/TT_cube_giga.pt",
        "Bolt": "./scripts/data/models/TT_boltM24_giga.pt",
        "Sundries":"./scripts/data/models/Sundries_giga.pt"
    }

#內部參數
# intrinsic=np.array([[606.77737126, 0., 321.63287183],[0., 606.70030146, 236.95293136],[0., 0., 1.]])
# distortion=np.array([4.90913179e-02 , 5.22699002e-01, -2.65209452e-03  ,1.13033224e-03,-2.17133474e+00])
with open(str(CALIBRATION_PARAMS_SAVE_DIR / 'calibration_params.json')) as f:
    calibration_params = json.load(f)
    T_Camera2Gripper = np.array(calibration_params["T_cam2gripper"])
    intrinsic=np.array(calibration_params["intrinsic_matrix"])
    distortion=np.array(calibration_params["distortion_coefficients"])
#手臂速度
SPEED=1000
#TODO Joint位置 T + 52680 after install bin on flang
INIT_JOINT_CONFIGURE = [
    -126330,
    -18970,
    14748,
    -1553,
    -101441,
    46069 + 52680
]
#貨架中間前方位置
FRAME_CENTER_CONFIGURE = [144639,-36455,-25636,68992,-10800,59514]


#ArUco 拍攝位置joint id編號為1,2,3,4
pos=[[174375,-54,16533,62768,-30326,65378],
      [109160,-6946,10866,-42555,-20866,128429],
      [110836,4460,-46832,-147865,-38785,197236],
      [176587,13056,-41194,129422,-43470,14816]]
id_index=0 #id編號





# if(object_name=="TT_cube" or object_name=="Sundries"):
#     PLACE_JOINTS={
#         0:[-15662,16560,15707,1585,-81132,58007],
#         1:[11886,18580,17724,2187,-81985,47525]
#     }
#     PLACE_NUMBER=2

# else:
#     PLACE_JOINTS={
#     0:[-31240,232,-5933,1585,-75117,62778],
#     1:[-2031,-6852,-11616,2298,-75942,51623],
#     2:[33008,1119,-4909,2795,-77048,38245],
#     3:[57802,21806,14560,2865,-79469,28863],
#     4:[-25384,24541,17428,1739,-77462,60582],
#     5:[1089,19755,12450,2357,-77384,50459],
#     6:[27054,28742,21971,2737,-79619,40613],
#     7:[48653,49257,46394,2875,-85597,32569]
#     }
#     PLACE_NUMBER=8

#Rz轉90度的旋轉矩陣以及下爪深度
theta = np.radians(90)
T_gripper_Rz_90 = np.array([    
    [np.cos(theta), -np.sin(theta), 0, 0],
    [np.sin(theta), np.cos(theta), 0, 0],
    [0, 0, 1, OFFSET[object_name]],
    [0, 0, 0, 1]
])
# if object_name=="Block":
#     T_gripper_Rz_90 = np.array([    
#     [np.cos(theta), -np.sin(theta), 0, 0],
#     [np.sin(theta), np.cos(theta), 0, -20],
#     [0, 0, 1, OFFSET[object_name]],
#     [0, 0, 0, 1]
# ])
#TODO校正參數
CALIBRATION_PARAMS_SAVE_DIR = Path(__file__).parent / "calibration_params/2024_06_13" 
FRAME_COORD_SAVE_DIR=Path(__file__).parent / "ArUco_Frame" 
# 工作空間大小單位是m
WorkSpaceSize = 0.3

#TODO表示相機座標到aruco的座標轉換
ArUco_json_filename = "./scripts/quaternion.json"
with open(ArUco_json_filename, 'r') as f:
    loaded_data = json.load(f)
PLACE_HEIGHT=0.03 #平台墊高幾m
tvec = loaded_data["tvec"]
tvec[2]+=PLACE_HEIGHT #代表整個平台往上墊高五公分避免背景跟worksapce在同一層變成無法判斷是邊界還是物體 
quaternion = loaded_data["quaternion"]
T_ArUco2Cam_m = Transform(Rotation.from_quat(quaternion), tvec)
print(tvec)
print(quaternion)

#控制手臂的執行緒
class ArmThread(QThread):
    release_trigger = pyqtSignal(object)
    grasping_trigger = pyqtSignal(object)
    def __init__(self):
        super(ArmThread, self).__init__()
        self._mutex = QMutex()
        self.pose = None
        self._running = False
        self.cntrl = Controller()
        self.grasping_trigger.emit(self.cntrl.robot_info)
        self.grip = Gripper()
    def initArmGripper(self):
        self.grip.gripper_reset()
        self.cntrl.power_on()
        self.grip.gripper_on()
    def run(self):
        while self.running():
            if self.pose == None:
                self.grasping_trigger.emit(self.cntrl.robot_info)
                self.stopGrasp()
            else:
                self.pick(self.pose)
                self.pose = None
    def pick(self,pose):
        pass

    def testArm(self):
        self.cntrl.get_robot_pos()
        pos = self.cntrl.robot_info
        print("pos=", pos)
        return self.cntrl.get_robot_pos()

    def calPosXY(self, camera_point):
        # print(camera_point)
        camera_xy = [camera_point[0], camera_point[1]]
        camera_xy.append(1.0)
        # print(camera_point)
        arm_point = np.array(np.array(camera_xy)) @ self.trans_mat
        print("arm_point", arm_point)
        return arm_point

    def reGrasp(self):
        # print("restart streaming")
        self._mutex.lock()
        self._running = True
        self._mutex.unlock()
        self.run()

    def stopGrasp(self):
        self.goHome()
        self._mutex.lock()
        self._running = False
        self._mutex.unlock()

    def running(self):
        try:
            self._mutex.lock()
            return self._running
        finally:
            self._mutex.unlock()

#使用模型計算夾取位置用
class GraspController(object):
    def __init__(self, args):
        
        self.finger_depth=0.05
        self.size = WorkSpaceSize #workspace空間
        self.tsdf_server = TSDFServer()
        self.plan_grasps = VGNImplicit(model_path=MODEL_PATH[args.model],
                                       model_type="giga", 
                                       best=True, 
                                       force_detection=True,
                                      )
        self.count=0
        print("!!!!!!"+MODEL_PATH[args.model])
        rospy.loginfo("Ready to take action")

    def run(self, robot_arm: ArmThread, T_Camera2Gripper: np.array, grasp_object=False,palce_location="table")->bool:
        
        robot_arm.grip.gripper_on()
        vis.clear()
        vis.draw_workspace(self.size)
        logging.info("[Robot] Move to initial pose by joint configure")
        
        tsdf, pc = self.acquire_tsdf(robot_arm,T_Camera2Gripper)
    
        state = State(tsdf, pc) 
        grasps, scores, planning_time= self.plan_grasps(state)
        #vis.draw_grasps(grasps, scores, self.finger_depth)
       
        rospy.loginfo("Planned grasps")

        if len(grasps) == 0:
            rospy.loginfo("No grasps detected")
            return True
        
        rospy.loginfo("Selected grasp")
        grasp, score = self.select_grasp(grasps, scores)
        vis.draw_grasp(grasp, score, self.finger_depth)
        

        rospy.loginfo("Grasp execution")
        
        if palce_location == "table" and grasp_object:
            T_grasp2task_m = grasp.pose
            print("模型測出來的位置")
            print(type(T_grasp2task_m))
            print(T_grasp2task_m.translation)
            #T_base_grasp = self.T_base_task
            
            print("grasp tsdf位置",grasp)
            
            T_Grasp2Base=self.EstimateCoord(T_grasp2task_m, T_Camera2Gripper, robot_arm, visual=False)
            robot_arm.grip.gripper_on()

            robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE,SPEED)
            #pregrasp_offset= -50 #mm
            pregrasp_offset= -30 #mm
            #移動到前方
            T_Grasp_Pregrasp = np.array([ 
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, pregrasp_offset],
                [0, 0, 0, 1]
            ])
            T_Base_Pregrasp = T_Grasp2Base @ T_Grasp_Pregrasp

            # TODO: 夾取座標後退
            Base_point_t = T_Base_Pregrasp[:3, 3] #3x1 T
            Base_point_r = T_Base_Pregrasp[:3,:3] #3x3
            Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=Base_R_degree[0],
                Ry=Base_R_degree[1],
                Rz=Base_R_degree[2],
                speed=SPEED)

            ## TODO: 夾取座標
            Base_point_t = T_Grasp2Base[:3, 3] #3x1 T
            Base_point_r = T_Grasp2Base[:3,:3] #3x3
            Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=Base_R_degree[0],
                Ry=Base_R_degree[1],
                Rz=Base_R_degree[2],
                speed=SPEED)
            #TODO: gripper
            robot_arm.grip.gripper_off()
            time.sleep(1)

            # TODO: 夾取座標後退
            Base_point_t = T_Base_Pregrasp[:3, 3] #3x1 T
            Base_point_r = T_Base_Pregrasp[:3,:3] #3x3
            Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=Base_R_degree[0],
                Ry=Base_R_degree[1],
                Rz=Base_R_degree[2],
                speed=SPEED)
            
            robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE, SPEED)  #初始位置
            # robot_arm.move_to_joint_config(Place_JOINT_CONFIGURE, SPEED) 
            #擺放位置
            robot_arm.cntrl.move_robot_joint(PLACE_JOINTS[self.count],SPEED)
            rospy.sleep(1)
            self.count = self.count+1
            if self.count == PLACE_NUMBER:
                self.count = 0 
            
            robot_arm.grip.gripper_on()
            robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE, SPEED)  #初始位置

        elif palce_location == "frame" and grasp_object:
            T_grasp2task_m = grasp.pose
            print("模型測出來的位置")
            print(type(T_grasp2task_m))
            print(T_grasp2task_m.translation)
            #T_base_grasp = self.T_base_task
            
            print("grasp tsdf位置",grasp)
            
            T_Grasp2Base=self.EstimateCoord(T_grasp2task_m, T_Camera2Gripper, robot_arm, visual=False)
            robot_arm.grip.gripper_on()

            robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE,SPEED)
            #pregrasp_offset= -50 #mm
            pregrasp_offset= -30 #mm
            #移動到前方
            T_Grasp_Pregrasp = np.array([ 
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, pregrasp_offset],
                [0, 0, 0, 1]
            ])
            T_Base_Pregrasp = T_Grasp2Base @ T_Grasp_Pregrasp

            # TODO: 夾取座標後退
            Base_point_t = T_Base_Pregrasp[:3, 3] #3x1 T
            Base_point_r = T_Base_Pregrasp[:3,:3] #3x3
            Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=Base_R_degree[0],
                Ry=Base_R_degree[1],
                Rz=Base_R_degree[2],
                speed=SPEED)

            ## TODO: 夾取座標
            Base_point_t = T_Grasp2Base[:3, 3] #3x1 T
            Base_point_r = T_Grasp2Base[:3,:3] #3x3
            Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=Base_R_degree[0],
                Ry=Base_R_degree[1],
                Rz=Base_R_degree[2],
                speed=SPEED)
            #TODO: gripper
            robot_arm.grip.gripper_off()
            time.sleep(1)


            # TODO: 夾取座標後退
            Base_point_t = T_Base_Pregrasp[:3, 3] #3x1 T
            Base_point_r = T_Base_Pregrasp[:3,:3] #3x3
            Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=Base_R_degree[0],
                Ry=Base_R_degree[1],
                Rz=Base_R_degree[2],
                speed=SPEED)
            
            robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE, SPEED)  #初始位置
            # robot_arm.move_to_joint_config(Place_JOINT_CONFIGURE, SPEED) 

            #擺放位置
            with open(str(FRAME_COORD_SAVE_DIR / 'Grid_pos.json')) as f:
                pos_list = json.load(f)
                print(pos_list[0])

            robot_arm.cntrl.move_robot_joint(FRAME_CENTER_CONFIGURE,SPEED)

            global id_index
            robot_arm.cntrl.move_robot_pos(
                Tx=pos_list[id_index][0],
                Ty=pos_list[id_index][1]-90, #往格子裡面6公分
                Tz=pos_list[id_index][2],
                Rx=pos_list[id_index][3],
                Ry=pos_list[id_index][4],
                Rz=pos_list[id_index][5],
                speed=SPEED)
            
            robot_arm.cntrl.move_robot_pos(
                Tx=pos_list[id_index][0],
                Ty=pos_list[id_index][1]+110, #往格子裡面11公分
                Tz=pos_list[id_index][2],
                Rx=pos_list[id_index][3],
                Ry=pos_list[id_index][4],
                Rz=pos_list[id_index][5],
                speed=SPEED)
            
            robot_arm.cntrl.move_robot_pos(
                Tx=pos_list[id_index][0],
                Ty=pos_list[id_index][1]+110, 
                Tz=pos_list[id_index][2]-40, #向下4公分
                Rx=pos_list[id_index][3],
                Ry=pos_list[id_index][4],
                Rz=pos_list[id_index][5],
                speed=SPEED)
            robot_arm.grip.gripper_on()

            robot_arm.cntrl.move_robot_pos(
                Tx=pos_list[id_index][0],
                Ty=pos_list[id_index][1]-30, #往後9公分
                Tz=pos_list[id_index][2]-40, #向下4公分
                Rx=pos_list[id_index][3],
                Ry=pos_list[id_index][4],
                Rz=pos_list[id_index][5],
                speed=SPEED)
            
            #robot_arm.cntrl.move_robot_joint(FRAME_CENTER_CONFIGURE,SPEED)
            robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE, SPEED)  #初始位置
            if id_index == 3:
                return True
            else:
                id_index = id_index+1

        rospy.sleep(1)
        logging.info("[Robot] Move to initial pose by joint configure")
        return False
        
    #移動去其他視角拍照
    def acquire_tsdf(self,robot_arm: ArmThread, T_Camera2Gripper: np.array)-> Tuple[TSDFVolume, o3d.cpu.pybind.geometry.PointCloud]: 
        self.tsdf_server.reset()
        #TODO 斜座標計算 --------------------
        global T_ArUco2Cam_m  
        T_ArUco2Base= self.Cal_T_ArUco2Base(robot_arm, T_Camera2Gripper) 

        #TODO 測試中
        T_TiltedGripper2Base,T_ArUco2Cam_m = self.spherical_to_cartesian(T_Camera2Gripper, T_ArUco2Base, phi=0, visual=False)
        #TODO 移動到斜的拍照位置 test中目前用來測試??但
        Base_point_t = T_TiltedGripper2Base[:3, 3] #3x1 T
        Base_point_r = T_TiltedGripper2Base[:3,:3] #3x3
        Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
        robot_arm.cntrl.move_robot_pos(
            Tx=Base_point_t[0],
            Ty=Base_point_t[1],
            Tz=Base_point_t[2], #保險起見避免撞到
            Rx=Base_R_degree[0],
            Ry=Base_R_degree[1],
            Rz=Base_R_degree[2],
            speed=SPEED)
        
        self.tsdf_server.integrate = True
        rospy.sleep(0.1)
        self.tsdf_server.integrate = False
        
        tsdf = self.tsdf_server.low_res_tsdf
        pc = self.tsdf_server.high_res_tsdf.get_cloud()
        mesh = self.tsdf_server.high_res_tsdf._volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        #o3d.visualization.draw_geometries([mesh])
        vis.draw_tsdf(tsdf.get_grid().squeeze(), tsdf.voxel_size)
        vis.draw_points(np.asarray(pc.points))
        rospy.loginfo("Reconstructed scene")
        print("-----------------")
        print(pc)  
        print("-----------------")

        robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE,SPEED)

        #TODO表示相機座標到aruco的座標轉換
        ArUco_json_filename = "./scripts/quaternion.json"
        with open(ArUco_json_filename, 'r') as f:
            loaded_data = json.load(f)
        tvec = loaded_data["tvec"]
        quaternion = loaded_data["quaternion"]
        tvec[2]+= PLACE_HEIGHT
        T_ArUco2Cam_m = Transform(Rotation.from_quat(quaternion), tvec)

        self.tsdf_server.integrate = True
        rospy.sleep(0.1)
        self.tsdf_server.integrate = False
        tsdf = self.tsdf_server.low_res_tsdf
        pc = self.tsdf_server.high_res_tsdf.get_cloud()
        mesh = self.tsdf_server.high_res_tsdf._volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        #o3d.visualization.draw_geometries([mesh])
        return tsdf, pc

    def select_grasp(self, grasps, scores):
        # select the highest grasp
        heights = np.empty(len(grasps))
        for i, grasp in enumerate(grasps):
            heights[i] = grasp.pose.translation[2]
        idx = np.argmax(heights)
        grasp, score = grasps[idx], scores[idx]

        # make sure camera is pointing forward
        rot = grasp.pose.rotation
        axis = rot.as_matrix()[:, 0]
        if axis[0] < 0:
            grasp.pose.rotation = rot * Rotation.from_euler("z", np.pi)

        return grasp, score
    

    def EstimateCoord(self,T_grasp2Aruco_m: Transform, T_Camera2Gripper: np.array, robot_arm: ArmThread, visual=False)-> np.array:  #計算座標
        tcp_pose = robot_arm.cntrl.get_robot_pos() #讀取手臂本身座標
        T_Gripper2Base=TcpPose_TO_Numpy(tcp_pose)
        print(f"TCP POSE mat: {T_Gripper2Base}")

        T_ArUco2Camera=Transform_TO_Numpy(T_ArUco2Cam_m)  # _m代表Transform 類別的物件
        T_Grasp2ArUco=Transform_TO_Numpy(T_grasp2Aruco_m)  # _m代表Transform 類別的物件
        print("T_Grasp2ArUco",T_Grasp2ArUco)

        #TODO顯示各個座標系
        if(visual==True):
            tm = TransformManager()
            tm.add_transform("grasp", "ArUco", T_Grasp2ArUco)
            tm.add_transform("gripper", "robot", T_Gripper2Base)
            tm.add_transform("camera", "gripper", T_Camera2Gripper)
            tm.add_transform("Aruco", "camera", T_ArUco2Camera)
            ax = tm.plot_frames_in("Aruco", s=100)
            ax.set_xlim((-1000, 1000))
            ax.set_ylim((-1000, 1000))
            ax.set_zlim((-1000, 1000))
            plt.show() #顯示座標圖
        
        
        #TODO base座標計算從右邊往左看,相機座標到夾爪座標再到base座標
        T_Grasp2Base = T_Gripper2Base @ T_Camera2Gripper @ T_ArUco2Camera @ T_Grasp2ArUco @ T_gripper_Rz_90    
        print("T_Camera2Gripper", T_Camera2Gripper)
        print("T_Gripper2Base",T_Gripper2Base)
        print("T_ArUco2Camera",T_ArUco2Camera)
        
        return T_Grasp2Base

    #TODO 計算 T_ArUco2Base 代表ArUco到Base的轉換關係  
    def Cal_T_ArUco2Base(self,robot_arm: ArmThread, T_Camera2Gripper: np.array) -> np.array:
        #目標task到相機cam的矩陣T_cam_task
        T_ArUco2Camera = Transform_TO_Numpy(T_ArUco2Cam_m)
        #相機cam的矩陣到目標task
        tcp_pose = robot_arm.cntrl.get_robot_pos() #讀取手臂本身座標
        print("#讀取手臂本身座標")
        print(tcp_pose)
        T_Gripper2Base = TcpPose_TO_Numpy(tcp_pose)
        print(T_ArUco2Camera)
        print(T_Camera2Gripper)
        print(T_Gripper2Base)
        T_ArUco2Base = T_Gripper2Base @ T_Camera2Gripper @ T_ArUco2Camera    #夾爪轉90注意offset會往上7公分
        print(T_ArUco2Base)

        return T_ArUco2Base
    
    #TODO 還要檢查camera到arUco怪怪的
    def spherical_to_cartesian(self, T_Camera2Gripper:np.array, T_ArUco2Base:np.array, phi: float,visual=False) -> np.array:
        T_Gripper2Camera = np.linalg.inv(T_Camera2Gripper)
        #TODO 球形座標xyz計算
        theta = 25
        radius = 0.45  #單位是公尺
        phi = 0# 角度
        # 單位是m
        x =  radius * np.sin(np.deg2rad(theta)) * np.cos(np.deg2rad(phi)) 
        y =  radius * np.sin(np.deg2rad(theta)) * np.sin(np.deg2rad(phi)) 
        z =  radius * np.cos(np.deg2rad(theta))
        # 讓相機指向ArUco的旋轉向量 只能手動嘗試,使用的是角度
        r= (
            R.from_euler("z",phi , degrees=True)*
            R.from_euler("y",theta , degrees=True)*
            R.from_euler("z",-90, degrees=True)*
            R.from_euler("x",180, degrees=True)
        )
        rx,ry,rz = r.as_euler("xyz", degrees=True) 
        #這裡要取得的是以mm 為單位的 (x+ (WorkSpaceSize/2))先不加
        T_Camera2ArUco = np.eye(4)
        T_Camera2ArUco[:3, :3] = Rotation.from_euler("xyz", [rx,ry,rz], degrees=True).as_matrix()
        T_Camera2ArUco[:3, 3] = np.array([x+ (WorkSpaceSize/2),y+(WorkSpaceSize/2),z]).T 
        
        T_Camera2ArUco_wks = T_Camera2ArUco.copy()
        T_Camera2ArUco_wks[:3, 3] = np.array([T_Camera2ArUco_wks[0,3]*1000,T_Camera2ArUco_wks[1,3]*1000,T_Camera2ArUco_wks[2,3]*1000]).T #把它從m 轉成mm
        
        # T_TiltedGripper2Base 代表的是 workspace中心點因為加上了(WorkSpaceSize/2)以工作空間為中心
        T_TiltedGripper2Base = T_ArUco2Base @ T_Camera2ArUco_wks @ T_Gripper2Camera  #T_Camera2ArUco_wks 考慮可以拆成兩個去做
        print(T_TiltedGripper2Base)
        T_Camera2ArUco_wks[:3, 3] = np.array([T_Camera2ArUco_wks[0,3]/1000,T_Camera2ArUco_wks[1,3]/1000,T_Camera2ArUco_wks[2,3]/1000]).T #把它從m 轉成mm
        T_ArUco2Camera_new = np.linalg.inv(T_Camera2ArUco_wks)

        #轉成transform形式
        rotation_m = R.from_matrix(T_ArUco2Camera_new[:3, :3])  #rotation_m 是transform 類別
        quaternion = rotation_m.as_quat()
        T_ArUco2Camera_new_m = Transform(Rotation.from_quat(quaternion), [T_ArUco2Camera_new[0,3],T_ArUco2Camera_new[1,3],T_ArUco2Camera_new[2,3]])
        if visual == True:
            tm = TransformManager()
            tm.add_transform("ArUco", "Base", T_ArUco2Base)
            #tm.add_transform("Camera", "ArUco_wks", T_Camera2ArUco_wks)
            tm.add_transform("ArUco", "Camera", T_ArUco2Camera_new)
            #tm.add_transform("Camera", "Gripper", T_Camera2Gripper)
            tm.add_transform("TiltedGripper", "Base", T_TiltedGripper2Base)
            ax = tm.plot_frames_in("ArUco", s=100)
            ax.set_xlim((-1000, 1000))
            ax.set_ylim((-1000, 1000))
            ax.set_zlim((-1000, 1000))
            plt.show() #顯示座標圖
        return  T_TiltedGripper2Base, T_ArUco2Camera_new_m

    

class TSDFServer(object): #只會做一次
    def __init__(self):
        self.cam_frame_id = "camera_depth_optical_frame"
        self.cam_topic_name = "/camera/aligned_depth_to_color/image_raw"
        self.intrinsic = CameraIntrinsic(640,480,intrinsic[0][0],intrinsic[1][1],intrinsic[0][2],intrinsic[1][2])
        self.distortion= distortion #影像扭曲嚴重時修正使用,若幾乎沒有扭曲可以不用
        self.size = WorkSpaceSize #30cmworksapce    
        self.cv_bridge = cv_bridge.CvBridge()
        self.tf_tree = ros_utils.TransformTree()
        self.integrate = False
        rospy.Subscriber(self.cam_topic_name, sensor_msgs.msg.Image, self.sensor_cb)
        
    def reset(self):
        self.low_res_tsdf = TSDFVolume(self.size, 40)
        self.high_res_tsdf = TSDFVolume(self.size, 120)
        
    def sensor_cb(self, msg: sensor_msgs.msg.Image):
        if not self.integrate:
            return

        img = self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32) * 0.001
        #T_ArUco2Cam_m =  ArUco_detection.Img_ArUco_detect(img,self.intrinsic,self.distortion)
        #用來廣播目標平面到相機的關係，也就是相機外參
        self.tf_tree.broadcast_static(
            T_ArUco2Cam_m, self.cam_frame_id, "task"
        )
        # T_ArUco2Cam_meter = T_ArUco2Cam_m
        self.low_res_tsdf.integrate(img, self.intrinsic, T_ArUco2Cam_m)
        self.high_res_tsdf.integrate(img, self.intrinsic, T_ArUco2Cam_m)

#
class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self, args ,parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        #--------------------Bind UI with func（Start）--------------------------------#代表對應的按鈕
        self.CameraOn.clicked.connect(self.Start2Stop) #CameraOn
        self.SaveImages.clicked.connect(self.SaveImagesF)  # Save Image
        self.GripperOpen.clicked.connect(self.GripperOpenF) #Open Gripper
        self.GripperClose.clicked.connect(self.GripperCloseF)  # Close Gripper
        self.ConnectRobtoArm.clicked.connect(self.ConnectRobotArmF) #connect RobotArm
        self.SetInitPos.clicked.connect(self.SetInitPosF) #Set Init Position
        self.GetPos.clicked.connect(self.GetPosF) #Get Current Position
        self.PickPlaceOntable.clicked.connect(self.Pick_PlaceOnTable) #Pick and place on table
        self.PickPlaceInFrame.clicked.connect(self.Pick_PlaceInFrame) #Pick and place on Frame
        self.PredictedNOpick.clicked.connect(self.Predicted_NOpick) #Only Predict 
        self.DetectArUcoPickPlane.clicked.connect(self.Img_ArUco_quaternion_detect) #Detect ArUco Pick Plane
        self.DetectArUcoPlacePlane.clicked.connect(self.Detect_four_ArUco) #Detect ArUco Place Plane
        # --------------------parameter------------------------------------------------
        self.cali_img_id=1 #相機校正影像編號
        self.intrinsic = np.empty((3,3))
        self.distCoeffs = np.empty((1,5))
        self.Camera2Gripper = np.empty((4,4))
        self.depth_value=0
        # --------------------Bind UI with func（End）--------------------------------
        self.robot_arm = ArmThread()
        # --------------------rviz set（start）-----------------------
        rospy.init_node("Yaskawa_grasp",anonymous=True)
        self.rgb_topic_name = "/camera/color/image_raw"
        self.depth_topic_name = "/camera/aligned_depth_to_color/image_raw"
        rospy.Subscriber(self.rgb_topic_name, sensor_msgs.msg.Image, self.rgb_callback)
        rospy.Subscriber(self.depth_topic_name, sensor_msgs.msg.Image, self.depth_callback)
        self.rgb_stream=False
        self.rgb_image = None
        self.depth_stream=False
        self.depth_image = None
        self.tf_tree = ros_utils.TransformTree()
        self.cv_bridge= cv_bridge.CvBridge()
        # 使用QTimer定期更新界面显示
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(30)  # 每30毫秒刷新一次
        #self.tsdf_server = TSDFServer()
        # --------------------rviz set（End）-------------------------
        #---------------------args-----------------------------------
        # parser = argparse.ArgumentParser()
        # parser.add_argument("--model", type=Path, default=MODEL_PATH[object_name])
        self.args = args
        #---------------------args (End)-----------------------------
    def Detect_four_ArUco(self):
        pos_list=[]
        position=[1,2,3,4]
        for index in position:

            self.robot_arm.cntrl.move_robot_joint(FRAME_CENTER_CONFIGURE,SPEED)
            self.robot_arm.cntrl.move_robot_joint(pos[index-1], SPEED)

            tvec, rvec,_ =Img_Place_ArUco_detect(self.rgb_image,intrinsic,distortion,Length = 22) #擺放的aruco邊長是mm 這裡使用毫米mm
            T_ArUco2Camera= np.eye(4)
            T_ArUco2Camera[:3, :3] = Rotation.from_rotvec(rvec).as_matrix()
            T_ArUco2Camera[:3, 3] = np.array(tvec[0]).T
            tcp_pose=self.robot_arm.cntrl.get_robot_pos()
            print(tcp_pose)
            T_Grasp2Base = Estimate_FramePlaceCoord(T_Camera2Gripper,tcp_pose,T_ArUco2Camera,index)
            
            print("T_Grasp2Base")
            print(T_Grasp2Base)
            Base_point_t = T_Grasp2Base[:3, 3] #3x1 T
            Base_point_r = T_Grasp2Base[:3,:3] #3x3
            #Base_R_degree= Rotation.from_matrix(Base_point_r).as_euler('xyz',degrees=True)
            print("Base_point_t")
            print(Base_point_t)
            self.robot_arm.cntrl.move_robot_pos(
                Tx=Base_point_t[0],
                Ty=Base_point_t[1],
                Tz=Base_point_t[2],
                Rx=89,
                Ry=0,
                Rz=175,
                speed=SPEED)
            pos_list.append([Base_point_t[0],Base_point_t[1],Base_point_t[2],89,0,175]) #儲存grid座標
            index+=1
            

        ArUco_json_filename = "./scripts/ArUco_Frame/Grid_pos.json"
        with open(ArUco_json_filename, 'w') as f:
            json.dump(pos_list, f)
        
    

    def Img_ArUco_quaternion_detect(self):
        data_dict = Img_Pick_ArUco_detect(self.rgb_image,intrinsic,distortion,Length = 0.035) #夾取的aruco邊長是35mm 這裡使用公尺meter
        self.OutPut.setText("quaternion(四元數): "+str(data_dict["quaternion"]))

    def Predicted_NOpick(self):
        self.robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE,SPEED)
        Yaskawa_grasp = GraspController(self.args)
        End = Yaskawa_grasp.run(self.robot_arm, T_Camera2Gripper,grasp_object=False,palce_location="None")  
            
    def Pick_PlaceInFrame(self):
        self.robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE,SPEED)
        Yaskawa_grasp = GraspController(self.args)
        while(True):
            End = Yaskawa_grasp.run(self.robot_arm, T_Camera2Gripper,grasp_object=True,palce_location="frame")
            if End:
                break

    def Pick_PlaceOnTable(self):
        self.robot_arm.cntrl.move_robot_joint(INIT_JOINT_CONFIGURE,SPEED)
        Yaskawa_grasp = GraspController(self.args)

        while(True):
            End = Yaskawa_grasp.run(self.robot_arm, T_Camera2Gripper,grasp_object=True,palce_location="table")  
            if End:
                break

    def rgb_callback(self, msg: sensor_msgs.msg.Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        self.rgb_image = cv_image.astype(np.uint8)

    def depth_callback(self, msg: sensor_msgs.msg.Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)#深度直需要化為0~255
        self.depth_image = cv_image.astype(np.uint8)

    def update_image(self):
        if self.rgb_image is not None and self.rgb_stream:
            height, width,channel = self.rgb_image.shape
            bytes_per_line =width*3
            rgb_image = QImage(self.rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.RGBFrame.setPixmap(QPixmap.fromImage(rgb_image).scaled(640, 480, Qt.KeepAspectRatio))
            self.RGBFrame.setScaledContents(True)

        if self.depth_image is not None and self.depth_stream:
            height, width = self.depth_image.shape
            bytes_per_line = width
            depth_image = QImage(self.depth_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
            self.DepthFrame.setPixmap(QPixmap.fromImage(depth_image).scaled(640, 480, Qt.KeepAspectRatio))
            self.DepthFrame.setScaledContents(True)

    def SetInitPosF(self):
        try:
            #self.robot_arm.move_to_joint_config[INIT_JOINT_CONFIGURE] 
            self.robot_arm.cntrl.move_robot_joint( INIT_JOINT_CONFIGURE, 1000) 
        except Exception as e:
            print(str(e))

    def GetPosF(self):
        try:
            self.OutPut.setText("pos="+str(self.robot_arm.testArm()))
        except Exception as e:
            print(str(e))

    def ConnectRobotArmF(self):
        try:
            if (self.ConnectRobtoArm.text() == "Connect Yaskawa"):
                self.robot_arm.initArmGripper()
                self.ConnectRobtoArm.setText("Disconnect Yaskawa")
                self.OutPut.append("Yaskawa Robot Arm Connected.")
                #self.updateGrasping()
            else:
                self.robot_arm.cntrl.power_off()
                self.ConnectRobtoArm.setText("Connect Yaskawa")
                self.OutPut.append("Yaskawa Robot Arm Disonnected.")
        except Exception as e:
            print(str(e))

    def Start2Stop(self):
            if(self.CameraOn.text() == "Camera ON"):
                self.CameraOn.setText("Camera OFF")
                print("Camera connected.")
                self.rgb_stream=True
                self.depth_stream=True
            else:
                self.CameraOn.setText("Camera ON")
                print("Camera disconnected.")
                self.rgb_stream=False
                self.depth_stream=False

    def SaveImagesF(self):
        try:
            if not os.path.exists(sys.path[0]+'/Saved_IMG'):
                os.makedirs(sys.path[0]+'/Saved_IMG')
            rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB) #因為opencv是bgr格式
            cv2.imwrite(sys.path[0]+"/Saved_IMG/RGB.png",rgb)
            cv2.imwrite(sys.path[0]+"/Saved_IMG/D.png",self.depth_image)
            self.OutPut.setText("Saved img")
        except Exception as e:
            self.OutPut.setText(str(e))

    def GripperOpenF(self):
        try:
            print("Open Gripper")
            self.robot_arm.grip.gripper_on()
        except Exception as e:
            self.OutPut.setText(str(e))

    def GripperCloseF(self):
        try:
            print("Close Gripper")
            self.robot_arm.grip.gripper_off()
        except Exception as e:
            self.OutPut.setText(str(e))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument("--model", type=Path, default=MODEL_PATH[object_name])
    parser.add_argument("--model", type=str, default="Block")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec_())
