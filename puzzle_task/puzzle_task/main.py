import copy
import math
import time
import sys
from calendar import c
from ctypes import *
from operator import mod

import cv2
import numpy as np
import pyrealsense2 as rs  # 版本目前不支援python3.10

sys.path.append("/home/weng/RoboticArm_Puzzle/main")
import Angle_Detect
import matlab
import YOLO_Detect

# '''
#     資料重現
# '''
# now_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
# save_file_path = '/home/weng/RoboticArm_Puzzle/src/libmodbus_ROS/src/main/save_data/'+str(now_time)
# file = open(save_file_path, 'w')
# # file.write(now_time+'\n')



"""
    設定
"""
# 手臂控制so檔
so_file = "/home/weng/RoboticArm_Puzzle/main/modbus_file/Hiwin_API.so"
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

# 手臂移動速度
speed_setting = 40
acceleration_setting = speed_setting

# 拼圖資訊
# 註記：拼圖狀態(-1:未發現,0:大略位置,1:精細位置與角度,2:已到拼圖區)
Puzzle = [[0 for i in range(3)] for j in range(35)]                  # [[X、Y], 角度, 拼圖狀態] * 35
for i in range(35):
    Puzzle[i][0] = [0,0]
    Puzzle[i][2] = -1

puzzle_imformation = [[0 for i in range(3)] for j in range(35)]      # [[X、Y], 角度, 拼圖狀態] * 35
for i in range(35):
    puzzle_imformation[i][0] = [0,0]
    puzzle_imformation[i][2] = -1

# 暫存拼圖狀態用
yolo_info = [[0 for i in range(2)] for j in range(35)]               # [[X、Y]] * 35
cv2_info  = []  # append

# 拼圖目標角度
#   -90
#    |
# 0--+--180
#    |
#    90
target_angle = [180.0, -90.0, 180.0, -90.0, 180.0, -90.0, 180.0,
                90.00, 0.000, 90.00, 180.0, 90.00, 180.0, 90.00,
                0.000, 90.00, 180.0, -90.0, 180.0, 90.00, 0.000,
                90.00, 180.0, -90.0, 0.000, -90.0, 0.000, 90.00,
                0.000, -90.0, 0.000, -90.0, 180.0, -90.0, 180.0,]    #目標角度

# 拼圖目標放置座標
target_coordinate = [[183.0, 76.5], [186.7,108.5], [182.7,144.0], [186.0,179.2], [183.3,214.9], [186.1,251.0], [182.6,282.7],
                     [216.0, 73.6], [216.7,108.2], [216.0,143.7], [216.0,179.6], [215.5,215.1], [216.0,251.1], [216.0,286.1],
                     [249.4, 76.5], [249.7,108.0], [249.5,143.5], [250.0,179.5], [249.6,215.4], [249.6,251.0], [249.9,282.6],
                     [283.5, 73.0], [283.6,108.0], [283.6,143.6], [283.4,179.2], [283.5,215.0], [283.6,250.9], [283.7,286.1],
                     [316.5, 76.5], [313.5,108.1], [316.6,143.6], [313.5,179.0], [316.9,215.1], [313.5,250.5], [317.5,282.5]]
# 8.13.26.31

# 拼圖順序
target_priority = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]           # 目標順序(預計)
Puzzle_priority = [0]*35                                                            # 拼圖順序(最終)


# 目前座標
Point_now = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]            # 預設為家座標

# 計算座標(下一個點)
Point_count = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]          # 預設為家座標

# 基本點位
Point_basic = [ [-18.681, 91.778, 261.325, -180, 0, 90],       # 0: 左收集區中點 #原pitch-179.091
                [410.202, 91.778, 261.325, -180, 0, 90],       # 1: 右收集區中點 #原pitch-179.091
                [250    , 68    , 450.1  , -180, 0, 90]]       # 2: 家

# 收集區長寬
Collect_length_width = [222, 310]                                  # (長、寬)

# 座標轉換比值
PIXEL2MM_all = 0.29277219                                           # 檢測所有拼圖位置時用(待測)


# 吸盤距離鏡頭(mm)
sucker2camera = [54.0, 32.519]                                     # (X、Y) #原49.335, 32.1

# 輸出(print)使用顏色
class bcolors:
    # 大項目
    Task = '\033[90m'       # 深藍
    Warning = '\033[31m'    # 紅

    # 資料計算
    Count = '\033[36m'      # 深綠
    OK = '\033[32m'         # 綠色加底
    Data = '\033[47m'       # 資料輸出

    # 手臂移動
    Move = '\033[93m'       # 黃色加底
    EndMove = '\033[32m'    # 綠色加底

    # 重設
    RESET = '\033[0m'       # RESET COLOR

'''
    拍照
'''
def get_picture():
    # img = cv2.imread("/home/weng/Downloads/15934122612690.jpg")
    frames = pipeline.wait_for_frames()
    img = frames.get_color_frame()
    img = np.asanyarray(img.get_data())

    return img

'''
    資料重現
'''
def record(word, time_mode, start_time):
    if time_mode=='start':
        start_time = time.time()
        # file.write(word)
    else:
        end_time = time.time()
        elapsed_time = end_time-start_time
        # file.write(word + str(elapsed_time) + '\n')
    
    return start_time

'''
    保存拼圖位置
'''
def save_coordinate(puzzle_imformation,mode='YOLO'):
    print(bcolors.Count + "    save coordinate... " + bcolors.RESET, end='')
    print(bcolors.Warning + "***" + bcolors.RESET)

    for i in range(35):
        # print(Puzzle)
        if puzzle_imformation[i][2]>-1:
            x, y = img_conversion(puzzle_imformation[i][0][0],puzzle_imformation[i][0][1],'ALL')
            Puzzle[i][0][0] = round(Point_now[0] - y + sucker2camera[0] ,3)                                 # 換算x座標並儲存
            Puzzle[i][0][1] = round(Point_now[1] - x + sucker2camera[1] ,3)                                 # 換算y座標並儲存
            Puzzle[i][2] = 0                                                                        # 更新拼圖狀態(已知大略位置)
            print("save_coordinate= ",Puzzle[i][0][0],Puzzle[i][0][1])
    print(bcolors.Warning + "    ***" + bcolors.RESET,end='')
    print(bcolors.OK + " OK" + bcolors.RESET)

'''
    影像轉換
    輸入: 影像'X'座標(單位:pixel), 影像'Y'座標(單位:pixel), 模式(detect or all or correction)
    輸出: 影像'X'座標(單位:mm), 影像'Y'座標(單位:mm) 
    更新全域變數: 無

    備註: 0點由左上改到中心
'''
'''
  4 |
x<------
    | 2
    y
'''
def img_conversion(x_img, y_img, mode='Angle'):
    x_gain = 1.006      # 基於手臂座標
    y_gain = 0.997
    
    x_gain2 = 1.000      # 基於手臂座標
    y_gain2 = 1.000
    x_gain4 = 1.000      # 基於手臂座標
    y_gain4 = 0.98
    print(bcolors.Count + "    img conversion... " + bcolors.RESET ,end='')
    print(bcolors.OK + "OK" + bcolors.RESET)
    print("img_conversion= ",round((x_img-(W_img/2))*PIXEL2MM_all, 3), round((y_img-(H_img/2))*PIXEL2MM_all, 3))
    
    x = ((x_img-(W_img/2))*PIXEL2MM_all)*y_gain
    y = ((y_img-(H_img/2))*PIXEL2MM_all)*x_gain

    if x>0:
        x*=-(y_gain4-1)+1
    else:
        x*=-(y_gain2-1)+1
    
    if y>0:
        y*=-(x_gain2-1)+1
    else:
        y*=-(x_gain4-1)+1


    return [round(x,3),round(y,3)]

'''
    拼圖優先度排序
'''
def Puzzle_sort():
    print(bcolors.Count + "    Puzzle sort... " + bcolors.RESET ,end='')
    num = 0
    for i in range(35):
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
def arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting):
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
def PTP_Move(Point, speed = speed_setting, acceleration = acceleration_setting):
    print(bcolors.Move + "    PTP Move ... " + bcolors.RESET ,end='')
    C_PTP_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.PTP(1, speed, acceleration, 0, 1, C_PTP_XYZ)
    modbus.Arm_State_REGISTERS()
    time.sleep(0.1)
    modbus.Arm_State_REGISTERS()
    modbus.Arm_State_REGISTERS()
    while 1:
        frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        frames.get_color_frame()                    # 同上
        if(modbus.Arm_State_REGISTERS() == 1):
            break
        time.sleep(0.01)
    print(bcolors.EndMove + "END PTP Move!" + bcolors.RESET)

'''
    LIN移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def LIN_Move(Point, speed = speed_setting, acceleration = acceleration_setting):
    print(bcolors.Move + "    LIN Move ... " + bcolors.RESET ,end='')
    C_LIN_XYZ = (c_double * len(Point))(*Point)         # C Array

    # modbus.LIN(1, speed, acceleration, 0, 1q, C_LIN_XYZ)
    # time.sleep(0.1)
    # while 1:
    #     frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
    #     frames.get_color_frame()                    # 同上
    #     if(modbus.Arm_State_REGISTERS() == 1):
    #         break
    #     # time.sleep(0.01)
    print(bcolors.EndMove + "END LIN Move!" + bcolors.RESET)





if __name__ == "__main__":

    # arm setting
    Arm_state = 0
    sucker_Port = 300       # 吸盤開關(D0)
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.LIN.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    modbus.DO(sucker_Port,0) # 1 -> on ; 0 -> off

    Point_count = Point_basic[2]
    arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
    arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)


    for area in range(0,2):
        '''
            設定移動目標
        '''
        Point_count = Point_basic[area]
        arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)


        '''
            拍照
        '''
        # img = cv2.imread('/home/weng/Downloads/puzzle_0021.jpg')
        img = get_picture()
        time.sleep(1)
        img = get_picture()

        '''
            YOLO檢測
            1. 輸入圖像進行檢測 >> 返回拼圖資訊(publackzzle_imformation)q
            2. 儲存資訊進Puzzle
            3. 排序拼回目標位置順序
        '''
        yolo_info = YOLO_Detect.detect_ALL(img)
        print("yolo_info= ",yolo_info)


        '''
            cv2檢測
        '''
        cv2_info = Angle_Detect.detect_ALL(img, k_gauss=13, low_threshold=10, high_threshold=20, k_close=4)
        print("cv2_info= ",cv2_info)


        '''
            比對YOLO&cv2結果
        '''
        i=0
        while 1:
            if yolo_info[i][0]>-999:
                for j in range(len(cv2_info)):
                    temp = math.sqrt(pow(yolo_info[i][0]-cv2_info[j][0][0],2) + pow(yolo_info[i][1]-cv2_info[j][0][1],2)).real
                    if temp<50:
                        print("i= ",i)
                        puzzle_imformation[i][0]=cv2_info[j][0]     # 座標
                        puzzle_imformation[i][1]=cv2_info[j][1]     # 角度
                        puzzle_imformation[i][2]=0                  # 狀態
            else:
                pass
            i+=1
            if i>=35:
                break


        '''
            找出正確角度
        '''
        yolo_info = YOLO_Detect.detect_Angle(img)
        # print("yolo_info",yolo_info)
        # print("cv2_info",cv2_info)
        print("puzzle_imformation",puzzle_imformation)
        for i in range(35):
            if yolo_info[i][0]>-999 and puzzle_imformation[i][0][0]>0:
                print(puzzle_imformation[i][0],yolo_info[i])

                # 因為圖像y軸相反(朝下)所以加負號將其倒回來
                YOLO_angle = round(math.degrees(math.atan2( -(yolo_info[i][1]-puzzle_imformation[i][0][1]) , yolo_info[i][0]-puzzle_imformation[i][0][0] )), 5)
                print("ooo=",i,YOLO_angle)

                compare_angle = [999,999]                                                                       #儲存兩個角度差
                allow_angle = 60
                cv2_angle = [999,999]
                cv2_angle[0] = puzzle_imformation[i][1][0]      # cv2角度1
                cv2_angle[1] = puzzle_imformation[i][1][1]      # cv2角度2

                if((YOLO_angle>(180-allow_angle)) and (cv2_angle[0]<(-180+allow_angle))) or ((YOLO_angle<(-180+allow_angle)) and (cv2_angle[0]>(180-allow_angle))):   #計算角度差1
                    compare_angle[0] = abs(YOLO_angle + cv2_angle[0])                            #兩角度介於正負180兩邊則以相加計算
                else:                                                                            #反之則以相減計算角度差
                    compare_angle[0] = abs(YOLO_angle - cv2_angle[0])
                if((YOLO_angle>(180-allow_angle)) and (cv2_angle[1]<(-180+allow_angle))) or ((YOLO_angle<(-180+allow_angle)) and (cv2_angle[1]>(180-allow_angle))):   #計算角度差2
                    compare_angle[1] = abs(YOLO_angle + cv2_angle[1])                            #兩角度介於正負180兩邊則以相加計算
                else:                                                                            #反之則以相減計算角度差
                    compare_angle[1] = abs(YOLO_angle - cv2_angle[1])

                if compare_angle[0] < compare_angle[1]:                                          #判斷兩個角度差何者差異較小
                    Puzzle[i][1] = cv2_angle[0]                                                  #將較符合的角度存入
                else:
                    Puzzle[i][1] = cv2_angle[1]

        
        '''
            儲存拼圖資訊
        '''
        save_coordinate(puzzle_imformation,mode='YOLO_ALL')

        puzzle_imformation = [[0 for i in range(3)] for j in range(35)]      # [[X、Y], 角度, 拼圖狀態] * 35
        for i in range(35):
            puzzle_imformation[i][0] = [0,0]
            puzzle_imformation[i][2] = -1

        print("Puzzle= ",Puzzle)


    '''
        可視化
    '''
    num=0
    for i in range(35): 
        if Puzzle[i][2]>-1:
            num+=1
    matlab.plt_(Puzzle,sucker2camera)

    # time.sleep(100000)

    for i in range(35):
        if Puzzle[i][2] > -1:

            # 吸盤移至拼圖上方
            print(bcolors.Task + "拼圖中... " + bcolors.RESET)
            Point_count[0] = Puzzle[i][0][0] # 設定X座標
            Point_count[1] = Puzzle[i][0][1] # 設定Y座標
            Point_count[2] = 25              # 設定Z座標
            Point_count[3] = 180
            Point_count[5] = 90
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)      # 移動手臂

            # 手臂下降
            Point_count[2] = 7       #手臂往下到吸取高度
            # Point_count[2] = 18
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            # break

            # 吸起拼圖
            modbus.DO(sucker_Port,1)    #吸起拼圖
            Point_count[2] = 25       #手臂往上到安全高度
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            

            # yaw角度計算
            arm_yaw = 90 + (target_angle[i] - Puzzle[i][1])
            if arm_yaw > 180:
                arm_yaw = arm_yaw - 360
            elif arm_yaw < -180:
                arm_yaw = arm_yaw + 360

            # yaw移動限制(去)
            arm_temp1 = 0
            arm_temp2 = 180
            if ((arm_yaw <= 0)and(arm_yaw > -120)):      # 假如逆時針轉太多,改成順時針先轉一點再過去
                Point_count[0] = (Point_count[0]+target_coordinate[i][0])/2
                Point_count[1] = (Point_count[1]+target_coordinate[i][1])/2
                Point_count[2] = 25
                Point_count[5] = arm_temp1
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            if (arm_yaw <= -120):                       # 假如逆時針轉太多,改成順時針先轉一點再過去
                Point_count[0] = (Point_count[0]+target_coordinate[i][0])/2
                Point_count[1] = (Point_count[1]+target_coordinate[i][1])/2
                Point_count[2] = 25
                Point_count[5] = arm_temp2
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            print('5')

            # 移至目標(拼圖)點
            Point_count[5] = arm_yaw    # 改牙軸
            Point_count[0] = target_coordinate[i][0]
            Point_count[1] = target_coordinate[i][1]
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            print('6')

            
            # 放置高度
            Point_count[2] = 10     #放置高度
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            

            # 左右小移動
            if(((i+1)%7==0) and ((1+1)!=35)):    #對右上(7,14,21,28)
                Point_count[0] += 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[0] -= 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] -= 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] += 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            elif(((i+1)>=29) and ((1+1)!=35)):   #對左下(29~34)
                Point_count[0] -= 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[0] += 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] += 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] -= 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            elif((1+1)==35):                     #對右下(35)
                Point_count[0] -= 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[0] += 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] -= 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] += 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            else:                                #對左上(剩下的)
                Point_count[0] += 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[0] -= 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] += 1.0
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

                Point_count[1] -= 1.6
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

            #放下拼圖
            modbus.DO(sucker_Port,0)
            time.sleep(0.5)
            Point_count[2] = 15     #手臂上升
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            # break

            # 再押一次
            Point_count[2] = 9     #下壓
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            Point_count[2] = 25     #回到安全高度
            arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

            # # yaw移動限制(下一片)
            arm_temp1 = 0
            arm_temp2 = 180
            if ((arm_yaw <= 0)and(arm_yaw > -120)):      # 假如逆時針轉太多,改成順時針先轉一點再過去
                for j in range(i+1,35):
                    if Puzzle[j][2] > -1:
                        break
                Point_count[0] = (Point_count[0]+Puzzle[j][0][0])/2
                Point_count[1] = (Point_count[1]+Puzzle[j][0][1])/2
                Point_count[2] = 25
                Point_count[5] = arm_temp1
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)
            if (arm_yaw <= -120):                       # 假如逆時針轉太多,改成順時針先轉一點再過去
                for j in range(i+1,35):
                    if Puzzle[j][2] > -1:
                        break
                Point_count[0] = (Point_count[0]+Puzzle[j][0][0])/2
                Point_count[1] = (Point_count[1]+Puzzle[j][0][1])/2
                Point_count[2] = 25
                Point_count[5] = arm_temp2
                arm_move('PTP', speed = speed_setting, acceleration = acceleration_setting)

        else:
            pass

Point_count = Point_basic[2]
arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
# time.sleep(1000000)
    
#-0.896 0.519
#37.687 120.484
#36.791 121.003
# file.close

    

    