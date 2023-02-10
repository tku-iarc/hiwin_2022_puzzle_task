import pyrealsense2 as rs           # 版本目前不支援python3.10
import cv2
from calendar import c
from operator import mod
import numpy as np
from ctypes import *
import time
import copy
import math

import Angle_Detect
import Correction
import YOLO_Detect
import matlab


"""
    設定
"""
# 手臂控制so檔
so_file = "/home/weng/HIWIN/src/libmodbus_ROS/src/My_test/Hiwin_API.so"
modbus = CDLL(so_file)

# Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)




"""
    全域變數
"""
# 相機拍照尺寸
H_img = 720
W_img = 1280

# 拼圖資訊
# 註記：拼圖狀態(-1:未發現,0:大略位置,1:精細位置與角度,2:已到拼圖區)
Puzzle = [[0 for i in range(3)] for j in range(21)]                                 # [[X、Y], 角度, 拼圖狀態] *21
for i in range(21):
    Puzzle[i][0] = [0,0]
    Puzzle[i][2] = -1

# 暫存拼圖狀態用
puzzle_imformation = [[0 for i in range(2)] for j in range(21)]                     # [[X、Y]] *21
target_angle = [180.0, 90.00, 180.0, 90.00, 180.0, 90.00, 00.00,
                -90.0, 180.0, -90.0, 180.0, -90.0, 180.0, -90.0,
                180.0, -90.0, 180.0, -90.0, 180.0, -90.0, 00.00,
                -90.0, 180.0, -90.0, 180.0, -90.0, 180.0, -90.0,
                180.0, -90.0, 180.0, -90.0, 180.0, -90.0, 00.00,]                      #目標角度

# 拼圖順序
target_priority = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]               # 目標順序(預計)
Puzzle_priority = [0]*21                                                                # 拼圖順序(最終)


# 目前座標
Point_now = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]            # 預設為家座標

# 計算座標(下一個點)
Point_count = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]          # 預設為家座標

# 基本點位
Point_basic = [ [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000],       # 0: 家
                [-262.891, 0397.663, 0115.898, 0180.000, 0000.046, 0090.003],       # 1: 左收集區
                [0165.239, 0397.754, 0115.813, -179.991, 0000.046, 0090.003],       # 2: 右收集區
                [-048.228, 0446.993, 0115.813, -179.991, 0000.046, 0090.003],       # 3: 拼圖區
                [-028.449, 0335.358, -028.294, -180.000, 0000.000, 0000.000]]       # 4: 影像校正區

# 收集區長寬
Collect_length_width = [222, 310]                                                   # (長、寬)

# 座標轉換比值
PIXEL2MM_correction = 0.1449275362                                                  # 座標校正時用(1pixel=0.144..mm)
PIXEL2MM_angle = 0.08823529412                                                      # 影像檢測時用(1pixel=0.088..mm)
PIXEL2MM_all = 0.29821074                                                           # 檢測所有拼圖位置時用(待測)

# 吸盤距離鏡頭(mm)
sucker2camera = [-47.0, -32.5]                                                      # (X、Y)

# 輸出(print)使用顏色
class bcolors:
    # 大項目
    Task = '\033[90m'    # 深藍
    Warning = '\033[31m'

    # 資料計算
    Count = '\033[36m'   #
    OK = '\033[32m'     #綠色加底
    Data = '\033[47m'   # 資料輸出

    # 手臂移動
    Move = '\033[93m' #黃色加底
    EndMove = '\033[32m' #綠色加底

    # 重設
    RESET = '\033[0m' #RESET COLOR




'''
    拍照
'''
def get_picture():
    frames = pipeline.wait_for_frames()
    img = frames.get_color_frame()
    img = np.asanyarray(img.get_data())

    return img


'''
    保存拼圖位置
'''
def save_coordinate(puzzle_imformation,mode='YOLO'):
    print(bcolors.Count + "    save coordinate... " + bcolors.RESET, end='')
    print(bcolors.Warning + "***" + bcolors.RESET)
    if mode=='YOLO':
        for i in range(21):
            if int(puzzle_imformation[i][0]) > -999:
                x, y = img_conversion(puzzle_imformation[i][0],puzzle_imformation[i][1],'YOLO')
                # print(x,y)
                Puzzle[i][0][0] = (Point_now[0] - y + sucker2camera[0])                                 # 換算x座標並儲存
                Puzzle[i][0][1] = (Point_now[1] - x + sucker2camera[1])                                 # 換算y座標並儲存
                Puzzle[i][2] = 0                                                                        # 更新拼圖狀態(已知大略位置)
    else:
        # print("puzzle_imformation= ",puzzle_imformation)
        for i in range(21):
            if int(puzzle_imformation[i][0]) > -999:
                x, y = img_conversion(puzzle_imformation[i][0],puzzle_imformation[i][1],'Angle')
                # print(x,y)
                Puzzle[i][0][0] = (Point_now[0] - y - sucker2camera[0])                                 # 換算x座標並儲存
                Puzzle[i][0][1] = (Point_now[1] - x - sucker2camera[1])                                 # 換算y座標並儲存
                Puzzle[i][2] = 1                                                                        # 更新拼圖狀態(已知角度)
    print(bcolors.Warning + "    ***" + bcolors.RESET,end='')
    print(bcolors.OK + " OK" + bcolors.RESET)



'''
    影像轉換
    輸入: 影像'X'座標(單位:pixel), 影像'Y'座標(單位:pixel), 模式(detect or all or correction)
    輸出: 影像'X'座標(單位:mm), 影像'Y'座標(單位:mm) 
    更新全域變數: 無

    備註: 0點由左上改到中心
'''
def img_conversion(x_img, y_img, mode='Angle'):
    print(bcolors.Count + "    img conversion... " + bcolors.RESET ,end='')
    print(bcolors.OK + "OK" + bcolors.RESET)
    if mode=='correction':  # 校正用
        return [round((x_img-(W_img/2))*PIXEL2MM_correction, 3), round((y_img-(H_img/2))*PIXEL2MM_correction, 3)]
    elif mode=='YOLO':          # 檢測大略位置(取物區)
        return [round((x_img-(W_img/2))*PIXEL2MM_all, 3), round((y_img-(H_img/2))*PIXEL2MM_all, 3)]
    else:                   # 檢測角度用
        return [round((x_img-(W_img/2))*PIXEL2MM_angle, 3), round((y_img-(H_img/2))*PIXEL2MM_angle, 3)]

'''
    座標轉換(影像座標轉為吸盤座標)
    輸入: 鏡頭上物體座標(mm)
    輸出: 無
    更新全域變數: Point_count[0]、Point_count[1]
'''
def coordinate_conversion(coordinate_puzzle):
    print(bcolors.Count + "    coordinate_conversion... " + bcolors.RESET ,end='')
    X = (Point_now[0] - coordinate_puzzle[1])          # X軸(目前X位置-影像Y位置(已轉mm)+吸盤與鏡頭X差距)
    Y = (Point_now[1] - coordinate_puzzle[0])          # Y軸
    print(bcolors.OK + "OK" + bcolors.RESET)

    return [X,Y]
    


def Puzzle_sort():
    print(bcolors.Count + "    Puzzle sort... " + bcolors.RESET ,end='')
    num = 0
    for i in range(21):
        if Puzzle[target_priority[i]-1][2]>-1:
            Puzzle_priority[num]=target_priority[i]
            num+=1
        else:
            pass
    print(bcolors.OK + "OK" + bcolors.RESET)

'''
    手臂移動
    輸入: 模式(PTP or LINE)、 速度、 加速度
    輸出: 無
    更新全域變數: Point_now
'''
def arm_move(mode='PTP', speed=50, acceleration=20):
    global Point_now

    if mode=='PTP':
        PTP_Move(Point_count, speed, acceleration)
    else:
        LIN_Move(Point_count, speed, acceleration)
    
    Point_now = copy.deepcopy(Point_count)

'''
    PTP移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def PTP_Move(Point, speed=50, acceleration=20):
    print(bcolors.Move + "    PTP Move ... " + bcolors.RESET ,end='')
    C_PTP_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.PTP(1, speed, acceleration, 1, 0, C_PTP_XYZ)
    modbus.Arm_State_REGISTERS()
    time.sleep(0.2)
    modbus.Arm_State_REGISTERS()
    modbus.Arm_State_REGISTERS()
    while 1:
        frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        frames.get_color_frame()                    # 同上
        if(modbus.Arm_State_REGISTERS() == 1):
            break
        # time.sleep(0.01)
    print(bcolors.EndMove + "END PTP Move!" + bcolors.RESET)

'''
    LIN移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def LIN_Move(Point, speed=50, acceleration=20):
    print(bcolors.Move + "    LIN Move ... " + bcolors.RESET ,end='')
    C_LIN_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.LIN(1, speed, acceleration, 1, 0, C_LIN_XYZ)
    time.sleep(0.2)
    while 1:
        frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        frames.get_color_frame()                    # 同上
        if(modbus.Arm_State_REGISTERS() == 1):
            break
        # time.sleep(0.01)
    print(bcolors.EndMove + "END LIN Move!" + bcolors.RESET)







if __name__ == "__main__":
    # arm setting
    Arm_state = 0
    sucker_Port = 301                                   # 吸盤開關(D0)
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.LIN.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    modbus.DO(sucker_Port,1) # 1 -> on ; 0 -> off
    

    '''
        座標校正
    '''
    # print(bcolors.Task + "座標校正... " + bcolors.RESET)
    # Point_count = Point_basic[4]                             # 設點位為影像校正區
    # arm_move('PTP', speed=50, acceleration=20)                # 移動手臂
    # while 1:
    #     if input()=='q':
    #         cv2.destroyAllWindows()
    #         break
    #     else:
    #         # img = cv2.imread('/home/weng/HIWIN/src/libmodbus_ROS/src/My_test/HIWIN_Puzzle_Program/test_data/555.png')
    #         time.sleep(0.1)
    #         img = get_picture()
    #         Angle_correction, coordinate_correction, out_img = Correction.detect(img)
    #         cv2.imshow("out_img",out_img)
    #         cv2.waitKey(100)
    # print(bcolors.OK + "座標校正完成" + bcolors.RESET)



    '''
        YOLO檢測
        1. 輸入圖像進行檢測 >> 返回拼圖資訊(publackzzle_imformation)q
        2. 儲存資訊進Puzzle
        3. 排序拼回目標位置順序
    '''
    print(bcolors.Task + "YOLO檢測... " + bcolors.RESET)

    old_Task = 0
    Task = 0
    while 1:
        if Task == 0:   # 移動至左收集區
            Point_count = Point_basic[1]                                                    # 設點位為左收集區
            arm_move('PTP', speed=50, acceleration=20)                                      # 移動手臂
            old_Task = Task
            Task = 2
        elif Task == 1: # 移動至右收集區
            Point_count = Point_basic[2]                                                    # 設點位為右收集區
            arm_move('PTP', speed=50, acceleration=20)                                      # 移動手臂
            old_Task = Task
            Task = 2
        elif Task == 2: # 進行YOLO檢測
            img = get_picture()                                                             # 拍照
            puzzle_imformation = YOLO_Detect.detect_ALL(img)                                # YOLO檢測
            old_Task = Task
            Task=1 if old_Task==0 else 3
        elif Task == 3: # 儲存拼圖資訊
            save_coordinate(puzzle_imformation, mode='YOLO')                                # 儲存拼圖資訊
            old_Task = Task
            Task = 4
        elif Task == 4: # 拼圖排序
            Puzzle_sort()                                                                   # 拼圖順序排序
            old_Task = Task
            Task = 5
        else:
            break
    
    num=0
    for i in range(21):                                                             # 計算有幾塊拼圖滿足條件加入排序
        if Puzzle[i][2]>-1:
            num+=1
    matlab.plt_(Puzzle)                                                             # 可視化
    print("\tFind \"{}\" piece >> {}".format(num,Puzzle_priority))

    
    # 左收集區
    Point_count = Point_basic[1]                                                    # 設點位為左收集區
    arm_move('PTP', speed=50, acceleration=20)                                       # 移動手臂
    time.sleep(0.1)
    # img = cv2.imread('/home/weng/Downloads/15934122612690.jpg')
    img = get_picture()
    puzzle_imformation = YOLO_Detect.detect_ALL(img)                                # YOLO檢測
    save_coordinate(puzzle_imformation, mode='YOLO')                                # 儲存拼圖資訊

    # 右收集區
    Point_count = Point_basic[2]                                                    # 設點位為左收集區
    arm_move('PTP', speed=50, acceleration=20)                                       # 移動手臂
    time.sleep(0.1)
    # img = cv2.imread('/home/weng/Downloads/15934122612690.jpg')
    img = get_picture()
    puzzle_imformation = YOLO_Detect.detect_ALL(img)                                # YOLO檢測
    save_coordinate(puzzle_imformation, mode='YOLO')                                # 儲存拼圖資訊

    Puzzle_sort()                                                                   # 拼圖順序排序

    num=0
    for i in range(21):                                                             # 計算有幾塊拼圖滿足條件加入排序
        if Puzzle[i][2]>-1:
            num+=1
    matlab.plt_(Puzzle)                                                             # 可視化
    print("\tFind \"{}\" piece >> {}".format(num,Puzzle_priority))



    '''
        角度檢測
        轉換並設定點位(大地>>攝影機),設定角度檢測高度
        移至該拼圖正上方
        輸入圖像進行檢測 >> 返回角度＆座標
        單位換算(pixel->mm)
        座標轉換(轉為大地座標)
        設定點位
        手臂移動
    '''
    # img = cv2.imread('/home/weng/yolo/pizzle_0217/train/puzzie_026.jpg')
    for i in range(21):
        now_puzzle_num = Puzzle_priority[i]

        if Puzzle_priority[i]!=0:
            print(bcolors.Task + "拼圖中... " + bcolors.RESET)
            print("    Targer >> {}:{}".format(Puzzle_priority[i],Puzzle[now_puzzle_num-1][0]))
            Point_count[0] = Puzzle[now_puzzle_num-1][0][0] - sucker2camera[0]                                                      # 設定座標(轉為攝影機位置)，X軸
            Point_count[1] = Puzzle[now_puzzle_num-1][0][1] - sucker2camera[1]                                                      # Y軸
            Point_count[2] = -76.550                                                                                                    # Z軸(檢測角度高度)
            Point_count[5] = 90
            arm_move('PTP', speed=50, acceleration=20)                                                                                   # 移動手臂
            
            # 第一次對正
            time.sleep(0.1)
            img = get_picture()
            Angle_puzzle, coordinate_puzzle = Angle_Detect.detect(img, k_gauss=17, low_threshold=10, high_threshold=20, k_close=7)      # 角度檢測
            print(Angle_puzzle)
            
            coordinate_puzzle = img_conversion(coordinate_puzzle[0],coordinate_puzzle[1],'Angle')                                      # 影像轉換
            coordinate_puzzle = coordinate_conversion(coordinate_puzzle)                                                                # 座標轉換
            Point_count[0] = coordinate_puzzle[0]                                                                                       # 設定座標
            Point_count[1] = coordinate_puzzle[1]
            arm_move('PTP', speed=50, acceleration=20)                                                                                   # 移動手臂


            # 正式檢測
            time.sleep(0.1)
            img = get_picture()                                                                                                               # 拍照
            Angle_puzzle, coordinate_puzzle = Angle_Detect.detect(img, k_gauss=17, low_threshold=10, high_threshold=20, k_close=7)      # 角度檢測
            for j in range(21):
                puzzle_imformation[j] = [-999,-999]
            puzzle_imformation[now_puzzle_num-1] = coordinate_puzzle
            save_coordinate(puzzle_imformation, mode='Angle')                                # 儲存拼圖資訊(座標)

            puzzle_imformation = YOLO_Detect.detect_Angle(img)
            YOLO_angle = round(math.degrees(math.atan2( -(puzzle_imformation[now_puzzle_num-1][1]-coordinate_puzzle[1]) , puzzle_imformation[now_puzzle_num-1][0]-coordinate_puzzle[0] )), 5)
            if abs(YOLO_angle - Angle_puzzle[0]) < abs(YOLO_angle - Angle_puzzle[1]):
                Puzzle[now_puzzle_num-1][1] = Angle_puzzle[0]
            else:
                Puzzle[now_puzzle_num-1][1] = Angle_puzzle[1]

            # 吸盤對準拼圖
            Point_count[0] = Puzzle[now_puzzle_num-1][0][0]                                                                         # 設定座標
            Point_count[1] = Puzzle[now_puzzle_num-1][0][1]
            arm_move('PTP', speed=50, acceleration=20)

            # 吸拼圖(上下動作)
            Point_count[2] = -134.5
            arm_move('PTP', speed=50, acceleration=20)
            time.sleep(0.1)
            Point_count[2] = -76.550
            arm_move('PTP', speed=50, acceleration=20)

            # yaw角度計算
            arm_yaw = 90 + (target_angle[now_puzzle_num-1] - Puzzle[now_puzzle_num-1][1])
            if arm_yaw > 180:
                arm_yaw = arm_yaw - 360

            # yaw移動限制(去)
            arm_temp = 180
            if arm_yaw < -60:                                   #假如逆時針轉太多,改成順時針先轉一點再過去
                Point_count[5] = arm_temp
                arm_move('PTP', speed=50, acceleration=20)
            Point_count[5] = arm_yaw                                                                          # 改牙軸
            arm_move('PTP', speed=50, acceleration=20)

            # yaw移動限制(返)
            if arm_yaw < -60:                                   #假如逆時針轉太多,改成順時針先轉一點再過去
                Point_count[5] = arm_temp
                arm_move('PTP', speed=50, acceleration=20)


            time.sleep(1)
            print(bcolors.OK + "角度檢測完成" + bcolors.RESET)


        else:
            Point_count = Point_basic[0]                                                    # 設點位為左收集區
            arm_move('PTP', speed=50, acceleration=20)                                       # 移動手臂
            print(bcolors.Task + "ALL END... " + bcolors.RESET)
            break





    

    
