import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import os

# from markerpos import markerpos



def rt_4x4(rvecs, tvecs): #把旋轉向量跟偏移向量合併成4x4的矩陣
    Rvecs = RotationTrans(rvecs)
    Tvecs = tvecs

    RPlanePos = Rvecs
    TPlanePos = Tvecs.reshape(3, 1)
    return np.r_[np.c_[RPlanePos, TPlanePos], [[0, 0, 0, 1]]]
    
def RotationTrans(v_org): #將手臂的由拉角(Euler angle)轉換為旋轉矩陣
    v=np.copy(v_org)

    pi = np.pi / 180

    # ================Epson===============  #Epson手臂才需要調換(x,y,z)裡的x跟z
    # tmp_v = v[0]
    # v[0] = v[2]
    # v[2] = tmp_v
    # ================Epson===============

    # pi =   1
    r1_mat = np.zeros((3, 3), np.float32)
    r2_mat = np.zeros((3, 3), np.float32)
    r3_mat = np.zeros((3, 3), np.float32)

    r = np.zeros((3, 1), np.float32)
    r[0] = 0
    r[1] = 0
    r[2] = float(v[2]) * pi
    r3_mat, jacobian = cv2.Rodrigues(r)
    r[0] = 0
    r[1] = float(v[1]) * pi
    r[2] = 0
    r2_mat, jacobian = cv2.Rodrigues(r)
    r[0] = float(v[0]) * pi
    r[1] = 0
    r[2] = 0
    r1_mat, jacobian = cv2.Rodrigues(r)

    result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    # print(result)
    return result


def isRotationMatrix(R):  #因為旋轉矩陣A的反矩陣即為轉置矩陣AT、A X AT =I，用乘出來的矩陣來判斷是否為單位矩陣 ，由於捨入誤差所以不能使用=0要用<1e-6
    Rt=np.transpose(R)
    shouldBeIdentity=np.dot(Rt,R)
    I=np.identity(3,dtype=R.dtype)
    n=np.linalg.norm(I-shouldBeIdentity)
    return n<1e-6

def rotationMatrixToEulerAngles(R) : #將旋轉矩陣轉換為手臂的旋轉座標(Euler angle)
    # assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    # ================Epson===============  #Epson手臂才需要調換(x,y,z)裡的x跟z
    # x,z=z,x
    # ================Epson===============
    
    return np.array([x, y, z])*180/np.pi




def eulerrot(x_rot=0,y_rot=0,z_rot=0):
    deg2rad = np.pi / 180
    x_rotmat = cv2.Rodrigues(np.array([x_rot * deg2rad, 0, 0]))[0]
    y_rotmat = cv2.Rodrigues(np.array([0, y_rot * deg2rad, 0]))[0]
    z_rotmat = cv2.Rodrigues(np.array([0, 0, z_rot * deg2rad]))[0]

    return np.dot(np.dot(x_rotmat,y_rotmat),z_rotmat)

def rotateuvw(pos_org,x_rot=0,y_rot=0,z_rot=0): 
    """
    #旋轉手臂角度並輸出尤拉角
    """
    if len(pos_org)==3:
        pos=np.copy(pos_org)
    if len(pos_org)==6:
        pos = np.copy(pos_org[3:])

    rotmat = eulerrot(x_rot,y_rot,z_rot)


    pos_mat = RotationTrans(pos)
    print('pos_mat',pos_mat)
    
    posrot_mat = np.dot(pos_mat,rotmat)
    print('posrot_mat',posrot_mat)
    posrot = rotationMatrixToEulerAngles(posrot_mat)

    if len(pos_org) == 6:
        posrot = np.append(pos_org[:3],posrot)
    print('posrot',posrot)

    return posrot






def calM2C(intr,img, id):  #此寫法僅適用opencv-contrib 4.6.0.66或更舊  前一屆學長因為相機較好FOV可以很小所以沒有處理可能同時出現多組id的情況
    # calculate marker2camera rt
    #mtx = readfile('intrinsic') 
    mtx = intr
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    markerCorners, markerIds, rejectedCandiates = detector.detectMarkers(img)
    # for i in range(len (markerCorners)):
    #     print(markerCorners[i])
    markerCenter= np.zeros((1,2) ,dtype=float)
    if id!=None:    #無用部分
        print(id)
        print(markerCenter)
        #markerCenter[0][0]=1
        id = np.where(markerIds==id)
        print(id[0])
        print(id[1])
        print(markerIds)
        print((np.squeeze(markerCorners[id[0][0]][id[1][0]])[0][0] + np.squeeze(markerCorners[id[0][0]][id[1][0]])[1][0] + np.squeeze(markerCorners[id[0][0]][id[1][0]])[2][0]+ np.squeeze(markerCorners[id[0][0]][id[1][0]])[3][0]) / 4)
        #markerCorners = markerCorners[id[0][0]][id[1][0]]      
        markerCenter[0][0]=((np.squeeze(markerCorners[id[0][0]][id[1][0]])[0][0] + np.squeeze(markerCorners[id[0][0]][id[1][0]])[1][0] + np.squeeze(markerCorners[id[0][0]][id[1][0]])[2][0]+ np.squeeze(markerCorners[id[0][0]][id[1][0]])[3][0]) / 4)
        markerCenter[0][1]=((np.squeeze(markerCorners[id[0][0]][id[1][0]])[0][1] + np.squeeze(markerCorners[id[0][0]][id[1][0]])[1][1] + np.squeeze(markerCorners[id[0][0]][id[1][0]])[2][1]+ np.squeeze(markerCorners[id[0][0]][id[1][0]])[3][1]) / 4)
        print("#####")
        print(markerCenter)
    #print(markerCorners[id[0][0]][id[1][0]])    

    #markerCenter = np.average(markerCorners[0], axis=1)
    print("wwww")
    print(markerCenter)
    #無用部分
    # markerCorners20=(markerCorners[0][0][2]-markerCorners[0][0][0])/12  #?
    # markerCorners31=(markerCorners[0][0][3]-markerCorners[0][0][1])/12

    # markerCorners_new=np.array([[markerCorners[0][0][0]+markerCorners20, #?
    #                             markerCorners[0][0][1]+markerCorners31,
    #                             markerCorners[0][0][2]-markerCorners20,
    #                             markerCorners[0][0][3]-markerCorners31]])

    print(markerCorners)
    # print(markerCorners_new)
    #畸變矩陣 我自己新增的
    distCoeffs=[ 7.6886595599879037e-02, 1.5764255689841356e-01,3.2915952106577710e-04, 4.2543524385500950e-03,-1.0918636433467268e+00]
    distCoeffs=np.array(distCoeffs)

    Marker2Camera_R, Marker2Camera_T, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.022, mtx, distCoeffs)  #檢查深度
    Marker2Camera = rt_4x4(Marker2Camera_R[0, 0], Marker2Camera_T[0, 0])
    print('Marker2Camera', Marker2Camera)
    print('Marker2Camera_R', Marker2Camera_R)

    img_axis = cv2.drawFrameAxes(np.copy(img),mtx,distCoeffs,Marker2Camera_R[0, 0],Marker2Camera_T[0, 0],0.01)  #[[[]]]因為像這樣包起來
    # cv2.imshow('img_axis',img_axis)
    # cv2.waitKey(0)

    return Marker2Camera, markerCenter, img_axis


def calM2B(Gripper2Base_R,Gripper2Base_T, Marker2Camera, depth):  #這是用來計算變換矩陣Maker to Base
    print('calM2B-----------------')
    # robot arm coordinate to transformation matrix
    
    Gripper2Base = rt_4x4(Gripper2Base_R, Gripper2Base_T)
    print('Gripper2Base', Gripper2Base)

    Camera2Gripper = readfile('C2G')  
    
    print('Camera2Gripper\n',Camera2Gripper)

    # get marker depth
    ###############################################  DEPTH  
    if type(depth) == float:
        print('Marker2Camera_org', Marker2Camera)
        Marker2Camera[2, 3] = depth  #手動測量目前這組
        #Marker2Camera[2, 3] = depth  #mm  因為座標變換矩陣變換是在夾爪開啟的時候中心點,因為用夾爪閉起來去點所以深度值會被增加一公分，
        #Marker2Camera[2, 3] = depth -10    #mm   原始版
        #Marker2Camera[2, 3] = Marker2Camera[2, 3] - 10  #純粹計算出來的深度(Y值)
        print('Marker2Camera', Marker2Camera)
        print('depth', depth)
    else:
        print('None')
        pass
    ###############################################

    # calculate marker2base rt
    Camera2Base = np.dot(Gripper2Base, Camera2Gripper)
    print('Camera2Base', Camera2Base)
    Marker2Base = np.dot(Camera2Base, Marker2Camera)
    print('Marker2Base', Marker2Base)
    
    Marker2Base_r = np.array([Marker2Base[0][:3], Marker2Base[1][:3], Marker2Base[2][:3]])
    Marker2Base_t = np.array([Marker2Base[0][3], Marker2Base[1][3], Marker2Base[2][3]])
    print('Marker2Base_r', Marker2Base_r)
    print('Marker2Base_t', Marker2Base_t)

    return Marker2Base_r, Marker2Base_t


def readfile(file):
    if file == 'intrinsic':
        fs = cv2.FileStorage('Extrinsic.txt', cv2.FILE_STORAGE_READ)
        fn = fs.getNode('intrinsic')
        return fn.mat()
    if file == 'C2G':
        fs = cv2.FileStorage('Extrinsic.txt', cv2.FILE_STORAGE_READ)
        fn = fs.getNode('Camera2Gripper')
        return fn.mat()


def ptsshiftinward(pts, frac): #??
    pts_inward = np.zeros((4, 2))
    pts_inward[0] = pts[0] + (pts[3] - pts[0]) * frac
    pts_inward[1] = pts[1] + (pts[2] - pts[1]) * frac
    pts_inward[2] = pts[2] + (pts[1] - pts[2]) * frac
    pts_inward[3] = pts[3] + (pts[0] - pts[3]) * frac
    pts_inward = np.int32(np.round(pts_inward))
    pts_average=np.int32(np.average(pts,axis=0))
    return pts_average, pts_inward

def draw_marker_graph(P,color):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    ax.set_zlabel('z (mm)')
    ax.set_xlim3d(-1000, 500)
    ax.set_ylim3d(0, 2000)
    ax.set_zlim3d(0, 1000)
    ax.view_init(20, -30)


    
    for p in range(len(P)):
        pts, c = P[p], color[p]
        for i in range(pts.shape[0]):
            ax.plot(pts[i, :, 0], pts[i, :, 1], pts[i, :, 2], c)

        for i in range(pts.shape[1]):
            ax.plot(pts[:, i, 0], pts[:, i, 1], pts[:, i, 2], c)

        pn = (-1)**(p%2)
        for i in range(pts.shape[0]):
            for j in range(pts.shape[1]):
                ax.scatter(pts[i, j, 0], pts[i, j, 1], pts[i, j, 2], color=c)
                label = '(' + str(round(pts[i, j, 0], 1)) + ', ' + str(round(pts[i, j, 1], 1)) + ', ' + str(
                    round(pts[i, j, 2], 1)) + ')'
                # print(label)
                s = 30  
                ax.text(pts[i, j, 0]+s, pts[i, j, 1], pts[i, j, 2]+s*pn, '%s' % label, size=8, zorder=10, color=c)

    path = 'savegraph'
    if not os.path.isdir(path):
        os.mkdir(path)
    plt.savefig('savegraph/plot.png')
    plt.show()

    img=cv2.imread('savegraph/plot.png')
    print(img.shape)
    img=img[100:424,50:626]
    print(img.shape)
    # cv2.imshow('',img)
    # cv2.waitKey(0)
    return img


# def capture4marker():  #有問題似乎被廢棄了

#     Marker2Base_r = [0, 0, 0, 0]
#     Marker2Base_t = [0, 0, 0, 0]

#     for i in range(4):
#         # move robot arm

#         Gripper2Base_R, Gripper2Base_T, depth = markerpos(i, 0)  #markerpos的function已被刪除
#         #Gripper2Base_R, Gripper2Base_T, depth = markerpos(i, 0)  
#         # capture marker
#         img = cv2.imread('marker' + str(i + 1) + '_image/marker' + str(i + 1) + '_1_Color.png')

#         # calculate marker
#         Marker2Base_r[i], Marker2Base_t[i] = calM2B(img, Gripper2Base_R, Gripper2Base_T, depth)

#     for i in range(4):
#         print('Marker' + str(i + 1))
#         print(Marker2Base_r[i])
#         print(Marker2Base_t[i])

#     # rotation matrix refinement
#     fcoor9x9 = np.array([[380, 0, 0, 0, 0, 0, 0, 0, 0],
#                          [0, 0, 0, 380, 0, 0, 0, 0, 0],
#                          [0, 0, 0, 0, 0, 0, 380, 0, 0],
#                          [0, -370, 0, 0, 0, 0, 0, 0, 0],
#                          [0, 0, 0, 0, -370, 0, 0, 0, 0],
#                          [0, 0, 0, 0, 0, 0, 0, -370, 0],
#                          [380, -370, 0, 0, 0, 0, 0, 0, 0],
#                          [0, 0, 0, 380, -370, 0, 0, 0, 0],
#                          [0, 0, 0, 0, 0, 0, 380, -370, 0]])
#     diff9x1 = np.append(np.append(Marker2Base_t[1] - Marker2Base_t[0], Marker2Base_t[3] - Marker2Base_t[0]),
#                         Marker2Base_t[2] - Marker2Base_t[0])
#     # pseudo inverse
#     rrefine9x1 = np.dot(np.linalg.pinv(fcoor9x9), diff9x1)
#     print(rrefine9x1)
#     Marker2Base_r_refined = rrefine9x1.reshape(3, 3)
#     print('Marker2Base_r_refined:\n', Marker2Base_r_refined)
#     Frame2Base = np.zeros((4, 4))
#     Frame2Base[:3, :3] = Marker2Base_r_refined
#     Frame2Base[:3, 3] = Marker2Base_t[0]
#     Frame2Base[3, 3] = 1
#     print(Frame2Base)

#     print('\nMarker 2 org')
#     print(Marker2Base_t[1])
#     print('Marker 2 refined')
#     print(np.dot(Frame2Base, [380, 0, 0, 1]))

#     print('\nMarker 3 org')
#     print(Marker2Base_t[2])
#     print('Marker 3 refined')
#     print(np.dot(Frame2Base, [380, -370, 0, 1]))

#     print('\nMarker 4 org')
#     print(Marker2Base_t[3])
#     print('Marker 4 refined')
#     print(np.dot(Frame2Base, [0, -370, 0, 1]))

class cal_4aruco: #測試用
    def __init__(self):
        self.marrow = 2  #aruco個數編號 0,1
        self.marcol = 2
        #'2883','-246016','166040','-1709973','-1929','-104740'
        self.b4markerpos = np.array([2883,-246016,166040,-1709973,-1929,-104740])  #初始位置?

        # self.fourmarkerpos = np.array([[[-232.469, 1082.936, 625.426, -26.735, 179.035, -25.241],
        #                                 [353.138, 1086.088, 628.608, -26.735, 179.035, -25.243]],
        #                                [[-218.092, 1086.089, -61.402, -26.703, 179.035, -25.211],
        #                                 [359.343, 1086.099, -55.169, -26.78, 179.035, -25.29]]])
        self.fourmarkerpos = np.array([[[160.627,502.145,500.945,-23.8362,-76.0695,-39.6246],             #yaskwa版本四個aruco座標
                                        [129.797,502.612,490.213,-31.2082,-76.3973,-64.4463]],
                                       [[-155.358,502.550,190.780,-119.3681,-79.0796,54.0192],
                                        [138.540,502.145,185.369,-122.2357,-78.6425,24.4526]]])
        
        self.frame4pos=np.array([[[95, 92.5, -3, 1],[285, 92.5, -3, 1]],[[95, 277.5, -3, 1],[285, 277.5, -3, 1]]])
        self.frame4marpos=np.array([[[0, 0, 0],[380, 0, 0]], [[0, 370, 0], [380, 370, 0]]])


        # self.gridrow = 4 # grid的四個角落
        # self.gridcol = 3 #?
        # camback = -200   #目前無用

        # self.camback = camback
        # self.capmgpos_F = np.array([[[97, 86, camback], [290, 86, camback], [483, 86, camback]],
        #                             [[97, 258, camback], [290, 258, camback], [483, 258, camback]],
        #                             [[97, 430, camback], [290, 430, camback], [483, 430, camback]],
        #                             [[97, 602, camback], [290, 602, camback], [483, 602, camback]]])
        # self.capmgpos_B = np.zeros(self.gridrow, self.gridcol, 6)
        # # fs = cv2.FileStorage('markermatrix/capmgpos_base.txt', cv2.FILE_STORAGE_READ)
        # # self.capmgpos_base = fs.getNode('capmgpos_base').mat()
        # up = 5
        # self.grasppos_F = np.array([[[97, 172 - up, 0], [290, 172 - up, 0], [483, 172 - up, 0]],
        #                             [[97, 344 - up, 0], [290, 344 - up, 0], [483, 344 - up, 0]],
        #                             [[97, 516 - up, 0], [290, 516 - up, 0], [483, 516 - up, 0]],
        #                             [[97, 688 - up, 0], [290, 688 - up, 0], [483, 688 - up, 0]]])
        # self.grasppos_B = np.zeros(self.gridrow, self.gridcol, 6)
        # fs = cv2.FileStorage('markermatrix/grasppos_base.txt', cv2.FILE_STORAGE_READ)
        # self.grasppos_base = fs.getNode('grasppos_base').mat()





        


        # self.fourmarkerpos=np.loadtxt('markermatrix/fourmarkerpos.txt')
        # self.grid_pos=np.loadtxt('markermatrix/grid_pos.txt')

        # self.grid_pos=np.zeros((2,2,6))

        fs = cv2.FileStorage('markermatrix/grid_pos.txt', cv2.FILE_STORAGE_READ)  #?
        self.grid_pos = fs.getNode('grid_pos').mat()



        # self.frame4pos=np.loadtxt('markermatrix/frame4pos.txt')
        self.Frame2Base_glob=np.loadtxt('markermatrix/Frame2Base_glob.txt')  #?

        self.place_Pos = np.array([[-521.3, 259.3, 220.5, 179.2, 0.968, 91.17], [-348.3, 259.3, 220.5, 179.2, 0.968, 91.17]])  #擺放座標
        

        

        # fs = cv2.FileStorage('markermatrix/markermatrix.txt', cv2.FILE_STORAGE_WRITE)
        # fs.write('fourmarkerpos', self.fourmarkerpos)
        # fs.write('grid_pos', self.grid_pos)
        # fs.write('frame4pos', self.frame4pos)
        # fs.write('Frame2Base_glob', self.Frame2Base_glob)

        fs = cv2.FileStorage('Extrinsic.txt', cv2.FileStorage_READ)  #要用我們的那一組
        c2g = fs.getNode('Camera2Gripper')
        self.c2g = c2g.mat()
        self.intr = fs.getNode('intrinsic').mat()
        fs.release()

        self.captureGridDistance = np.array([[None,335.25], [335.25, 335.25 - 50]])  #距離未知?
    
    def move_each_marker(self):
        marcol = self.marcol
        marrow = self.marrow


        Marker2Base_r = np.zeros((marrow, marcol, 3, 3))
        Marker2Base_t = np.zeros((marrow, marcol, 3))

        fourmarkerpos_copy = np.copy(self.fourmarkerpos)
        print('fourmarkerpos_copy\n', fourmarkerpos_copy)


        print('Start!')

        for i in range(marrow):
            j_range = range(marcol)
            if i % 2 != 0:
                j_range = range(marcol - 1, -1, -1)

            for j in j_range:
                # move to marker
                pos = fourmarkerpos_copy[i, j]
                print('marker ' + str(i) + str(j), pos[:3], pos[3:])
                Marker2Base_r[i, j], Marker2Base_t[i, j] = self.move2get_M2B(pos)   #因為分開拍

                
                print(Marker2Base_r[i, j], Marker2Base_t[i, j])
                print('pos',pos)

        self.robot.set_initPos()

        for i in range(marrow):
            for j in range(marcol):
                print('Marker' + str(i) + str(j))
                print(Marker2Base_r[i, j])
                print(Marker2Base_t[i, j])


        Marker2Base_t_vect = np.zeros((marrow, marcol, 3))

        Marker2Base_t_vect[0, 1] = Marker2Base_t[0, 1] - Marker2Base_t[0, 0]
        Marker2Base_t_vect[1, 0] = Marker2Base_t[1, 0] - Marker2Base_t[0, 0]
        crossvect00 = np.cross(Marker2Base_t_vect[0, 1], Marker2Base_t_vect[1, 0])  #法向量
        norvect00 = crossvect00 / np.sqrt(np.sum(crossvect00 ** 2))

        Marker2Base_t_vect[1, 1] = Marker2Base_t[1, 1] - Marker2Base_t[0, 0]

        Marker2Base_t11_10 = Marker2Base_t[1, 1] - Marker2Base_t[1, 0]
        Marker2Base_t11_01 = Marker2Base_t[1, 1] - Marker2Base_t[0, 1]
        crossvect11 = np.cross(Marker2Base_t11_10, Marker2Base_t11_01)
        norvect11 = crossvect11 / np.sqrt(np.sum(crossvect11 ** 2))

        norvect = np.array([(norvect00 + norvect11) / 2])

        Marker2Base_t_vect[1, 1] = Marker2Base_t[1, 1] - Marker2Base_t[0, 0]

        

        print('Marker2Base_t_01_00', Marker2Base_t_vect[0, 1])
        print('Marker2Base_t_10_00', Marker2Base_t_vect[1, 0])
        print('Marker2Base_t11_10', Marker2Base_t11_10)
        print('Marker2Base_t11_01', Marker2Base_t11_01)
        print('norvect00', norvect00)
        print('norvect11', norvect11)
        print('norvect', norvect)

        #svd
        Marker2Base_t_calrot =np.concatenate((Marker2Base_t_vect.reshape(marrow*marcol,3)[1:],norvect)).T  
        print('Marker2Base_t_calrot\n', Marker2Base_t_calrot)
        frame4marpos=np.copy(self.frame4marpos)
        Marker2Frame_t_calrot=np.concatenate((frame4marpos.reshape((frame4marpos.shape[0]*frame4marpos.shape[1],3))[1:],[[0,0,1]])).T
        # Marker2Frame_t_calrot = np.array([[380, 0, 0], [0, 370, 0], [380, 370, 0], [0, 0, 1]]).T
        print('Marker2Frame_t_calrot\n', Marker2Frame_t_calrot)
        Marker2Base_r_refined = np.dot(Marker2Base_t_calrot, np.linalg.pinv(Marker2Frame_t_calrot))
        print('Marker2Base_r_refined\n', Marker2Base_r_refined)
        u,v,w = np.linalg.svd(Marker2Base_r_refined)
        Marker2Base_r_refined = np.dot(u,w)
        Marker2Base_r_refined_uvw = rotationMatrixToEulerAngles(Marker2Base_r_refined)
        print('Marker2Base_r_refined_uvw', Marker2Base_r_refined_uvw)

        # ------------------------------------------------------------------------

        Frame2Base = np.zeros((4, 4))
        Frame2Base[:3, :3] = Marker2Base_r_refined
        Frame2Base[:3, 3] = Marker2Base_t[0,0]
        Frame2Base[3, 3] = 1
        print(Frame2Base)

        

        Marker2Base_r_gripper_uvw = rotateuvw(Marker2Base_r_refined_uvw,z_rot=-90)

        Marker2Base_t_refined = np.array([[Marker2Base_t[0,0],
                                           np.dot(Frame2Base, np.append(frame4marpos[0,1],1))[:3]],
        
                                          [np.dot(Frame2Base, np.append(frame4marpos[1,0],1))[:3],
                                           np.dot(Frame2Base, np.append(frame4marpos[1,1],1))[:3]]])

        print('\nMarker01 org')
        print(Marker2Base_t[0,1])
        print('Marker01 refined')
        print(np.dot(Frame2Base, np.append(frame4marpos[0,1],1)))
        print(Marker2Base_t_refined[0,1])

        print('\nMarker10 org')
        print(Marker2Base_t[1, 0])
        print('Marker10 refined')
        print(np.dot(Frame2Base, np.append(frame4marpos[1,0],1)))
        print(Marker2Base_t_refined[1,0])

        print('\nMarker11 org')
        print(Marker2Base_t[1,1])
        print('Marker11 refined')
        print(np.dot(Frame2Base, np.append(frame4marpos[1,1],1)))
        print(Marker2Base_t_refined[1,1])

        marker_graph = draw_marker_graph([Marker2Base_t_refined],['b'])
        # marker_graph = draw_marker_graph([Marker2Base_t],['b'])
        self.show_result_img(marker_graph)

        back = 20


        gridmove=np.zeros((2,2,4))

        print('\nGrid 00')
        gridmove[0,0] = np.dot(Frame2Base, self.frame4pos[0,0])
        print(gridmove[0,0])

        print('\nGrid 01')
        gridmove[0,1] = np.dot(Frame2Base, self.frame4pos[0,1])
        print(gridmove[0,1])

        print('\nGrid 10')
        gridmove[1,0] = np.dot(Frame2Base, self.frame4pos[1,0])
        print(gridmove[1,0])

        print('\nGrid 11')
        gridmove[1,1] = np.dot(Frame2Base, self.frame4pos[1,1])
        print(gridmove[1,1])

        self.robot.move_robotPos([gridmove[0,0][0], gridmove[0,0][1] - back, gridmove[0,0][2], Marker2Base_r_gripper_uvw[0],
                                  Marker2Base_r_gripper_uvw[1], Marker2Base_r_gripper_uvw[2]])
        self.robot.move_robotPos([gridmove[0,1][0], gridmove[0,1][1] - back, gridmove[0,1][2], Marker2Base_r_gripper_uvw[0],
                                  Marker2Base_r_gripper_uvw[1], Marker2Base_r_gripper_uvw[2]])
        self.robot.move_robotPos([gridmove[1,0][0], gridmove[1,0][1] - back, gridmove[1,0][2], Marker2Base_r_gripper_uvw[0],
                                  Marker2Base_r_gripper_uvw[1], Marker2Base_r_gripper_uvw[2]])
        self.robot.move_robotPos([gridmove[1,1][0], gridmove[1,1][1] - back, gridmove[1,1][2], Marker2Base_r_gripper_uvw[0],
                                  Marker2Base_r_gripper_uvw[1], Marker2Base_r_gripper_uvw[2]])


        

        self.robot.set_initPos()

        self.Frame2Base_glob = Frame2Base
        

        self.grid_pos[0,0] = np.array(
            [gridmove[0,0][0], gridmove[0,0][1], gridmove[0,0][2], Marker2Base_r_gripper_uvw[0], Marker2Base_r_gripper_uvw[1],
             Marker2Base_r_gripper_uvw[2]])
        self.grid_pos[0,1] = np.array(
            [gridmove[0,1][0], gridmove[0,1][1], gridmove[0,1][2], Marker2Base_r_gripper_uvw[0], Marker2Base_r_gripper_uvw[1],
             Marker2Base_r_gripper_uvw[2]])
        self.grid_pos[1,0] = np.array(
            [gridmove[1,0][0], gridmove[1,0][1], gridmove[1,0][2], Marker2Base_r_gripper_uvw[0], Marker2Base_r_gripper_uvw[1],
             Marker2Base_r_gripper_uvw[2]])
        self.grid_pos[1,1] = np.array(
            [gridmove[1,1][0], gridmove[1,1][1], gridmove[1,1][2], Marker2Base_r_gripper_uvw[0], Marker2Base_r_gripper_uvw[1],
             Marker2Base_r_gripper_uvw[2]])

        # np.savetxt('markermatrix/grid_pos.txt', self.grid_pos)
        cv2.FileStorage('markermatrix/grid_pos.txt', cv2.FILE_STORAGE_WRITE).write('grid_pos', self.grid_pos)
        np.savetxt('markermatrix/Frame2Base_glob.txt', self.Frame2Base_glob)


if __name__ == '__main__':
    # print(readfile('intrinsic'))
    aruco = cal_4aruco()
    aruco.move_each_marker()
    #capture4marker()原本的

    # print(RotationTrans(np.array([1.1378130340576172e+02, -8.8668296813964844e+01, 1.5591729736328125e+02])))

    # Marker2Base_t = np.array([[-3.3624650000000003e+02, 4.8613956250000001e+02, 7.5681956249999996e+02],
    #                           [-1.4668393750000001e+02, 4.7642896875000002e+02, 7.5591674999999998e+02],
    #                           [-3.3671659375000002e+02, 4.8398446875000002e+02, 5.7258256249999999e+02],
    #                           [-1.4715403125000000e+02, 4.7427387499999998e+02, 5.7167981250000003e+02]])
    #
    # fcoor9x9 = np.array([[380, 0, 0, 0, 0, 0, 0, 0, 0],
    #                      [0, 0, 0, 380, 0, 0, 0, 0, 0],
    #                      [0, 0, 0, 0, 0, 0, 380, 0, 0],
    #                      [0, -370, 0, 0, 0, 0, 0, 0, 0],
    #                      [0, 0, 0, 0, -370, 0, 0, 0, 0],
    #                      [0, 0, 0, 0, 0, 0, 0, -370, 0],
    #                      [380, -370, 0, 0, 0, 0, 0, 0, 0],
    #                      [0, 0, 0, 380, -370, 0, 0, 0, 0],
    #                      [0, 0, 0, 0, 0, 0, 380, -370, 0]])
    # print(fcoor9x9)
    # print(np.linalg.pinv(fcoor9x9))

    # diff9x1 = np.append(np.append(Marker2Base_t[1] - Marker2Base_t[0], Marker2Base_t[2] - Marker2Base_t[0]),
    #                     Marker2Base_t[3] - Marker2Base_t[0])
    # pseudo inverse
    # rrefine9x1 = np.dot(np.linalg.pinv(fcoor9x9), diff9x1)
    # print(rrefine9x1)
    # Marker2Base_r_refined = rrefine9x1.reshape(3, 3)
    # print(Marker2Base_r_refined)
    #
    # print('RotationTrans')
    # print(RotationTrans([1.1378130340576172e+02, -8.8668296813964844e+01, 1.5591729736328125e+02]))
